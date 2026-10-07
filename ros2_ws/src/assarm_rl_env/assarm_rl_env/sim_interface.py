# -*- coding: utf-8 -*-
"""
assarm_rl_env.sim_interface — Giao diện mô phỏng (SimEngine & ROS2 Bridge).

Hỗ trợ 2 chế độ:
1. Fast Simulation Engine (Thuần NumPy/Kinematics + ServoModel + Pendulum):
   - Đạt tốc độ > 200 step/s trên CPU i5-1035G4 cho huấn luyện SAC 100k–300k steps.
2. ROS2 Gazebo Bridge:
   - Kết nối với node Gazebo / ros_gz_bridge thông qua ROS2 topics.
"""

from typing import Tuple, Dict, Any
import numpy as np

from assarm_common.config import (
    DT_CTRL, DT_PHYS, SIM_SUBSTEPS, SPAWN_LEVELS,
    HANDOFF_EX, HANDOFF_EY, HANDOFF_EZ, HANDOFF_TILT, HANDOFF_REL_SPEED, HANDOFF_CONSEC_STEPS
)
from assarm_common.fk import fk_arm, p_close, gripper_tilt
from assarm_rl_env.servo_model import ServoModel


class FastSimEngine:
    """
    Trình mô phỏng vật lý & động học tốc độ cao cho Gymnasium Env.

    Mô phỏng động học cánh tay 4-DOF, mô hình phi tuyến servo ESP32,
    và chuyển động con lắc dây 30 cm của khối lập phương.
    """

    def __init__(self, dt: float = DT_CTRL):
        self.dt = dt
        self.servo_model = ServoModel(dt=dt)

        # Trạng thái tay & khối
        self.q_meas = np.zeros(4)
        self.q_target = np.zeros(4)
        self.gripper_meas = 0.0

        # Trạng thái khối con lắc
        self.cube_base_pos = np.array([0.12, 0.0, 0.20])
        self.swing_amp = 0.0
        self.swing_freq = 5.72  # rad/s (~0.91 Hz với L = 0.3m)
        self.swing_phase = 0.0
        self.cube_pos = self.cube_base_pos.copy()
        self.cube_vel = np.zeros(3)
        self.cube_yaw = 0.0

        # Bộ đếm step
        self.sim_time = 0.0
        self.consec_handoff_steps = 0

    def reset(self, spawn_level: int = 0, seed: int | None = None) -> Tuple[np.ndarray, np.ndarray]:
        """
        Reset trạng thái khối và robot về vị trí ban đầu.

        Parameters
        ----------
        spawn_level : int
            Cấp độ spawn (0..3). Stage A dùng Level 0-1 (swing = 0.0).
        seed : int, optional
            Hạt giống ngẫu nhiên.

        Returns
        -------
        q_init : ndarray(4)
            Góc khớp khởi tạo của tay.
        cube_pos_init : ndarray(3)
            Vị trí ban đầu của khối.
        """
        if seed is not None:
            np.random.seed(seed)

        params = SPAWN_LEVELS.get(spawn_level, SPAWN_LEVELS[0])

        r = np.random.uniform(params["r"][0], params["r"][1])
        theta = np.random.uniform(-params["theta"], params["theta"])

        z_min = 0.176 + 0.44 * (r - 0.10)
        z_max = 0.268 - 0.05 * (r - 0.10)
        z = np.clip(
            np.random.uniform(params["z"][0], params["z"][1]),
            z_min, z_max
        )

        # Tọa độ khối trong hệ base
        self.cube_base_pos = np.array([r * np.cos(theta), r * np.sin(theta), z])
        self.cube_yaw = np.random.uniform(-params["yaw"], params["yaw"])

        # Đung đưa
        self.swing_amp = params["swing"]
        self.swing_phase = np.random.uniform(0, 2 * np.pi)

        self.sim_time = 0.0
        self._update_cube_pose()

        # Góc khớp khởi tạo của robot hướng về phía khối
        y_j1 = -0.010797
        x_j1 = -0.000006
        q1_init = np.arctan2(self.cube_base_pos[1] - y_j1, self.cube_base_pos[0] - x_j1)
        self.q_meas = np.array([q1_init, 0.2, -0.2, 0.0])
        self.q_target = self.q_meas.copy()
        self.servo_model.reset(self.q_meas)
        self.gripper_meas = 0.0
        self.consec_handoff_steps = 0

        return self.q_meas.copy(), self.cube_pos.copy()

    def step(self, action_target: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, bool]:
        """
        Một bước mô phỏng (10 Hz).

        Parameters
        ----------
        action_target : ndarray(4)
            Target 4 khớp tay gửi từ policy.

        Returns
        -------
        q_meas : ndarray(4)
            Góc feedback khớp đo được từ servo model.
        cube_pos : ndarray(3)
            Vị trí thật của khối.
        cube_vel : ndarray(3)
            Vận tốc thật của khối.
        is_aligned : bool
            True nếu điều kiện bàn giao được thỏa mãn.
        """
        self.sim_time += self.dt
        self.q_target = action_target.copy()

        # Update servo response (vùng chết, rơ, trễ)
        self.q_meas = self.servo_model.step(self.q_target)

        # Update pendulum cube pose
        self._update_cube_pose()

        # Check alignment condition
        is_aligned = self._check_alignment()
        if is_aligned:
            self.consec_handoff_steps += 1
        else:
            self.consec_handoff_steps = 0

        handoff_ok = (self.consec_handoff_steps >= HANDOFF_CONSEC_STEPS)

        return self.q_meas.copy(), self.cube_pos.copy(), self.cube_vel.copy(), handoff_ok

    def _update_cube_pose(self) -> None:
        """Cập nhật vị trí & vận tốc khối con lắc."""
        if self.swing_amp <= 1e-5:
            self.cube_pos = self.cube_base_pos.copy()
            self.cube_vel = np.zeros(3)
        else:
            t = self.sim_time
            dx = self.swing_amp * np.cos(self.swing_freq * t + self.swing_phase)
            dy = self.swing_amp * np.sin(self.swing_freq * t + self.swing_phase)

            self.cube_pos = self.cube_base_pos + np.array([dx, dy, 0.0])

            vx = -self.swing_amp * self.swing_freq * np.sin(self.swing_freq * t + self.swing_phase)
            vy = self.swing_amp * self.swing_freq * np.cos(self.swing_freq * t + self.swing_phase)
            self.cube_vel = np.array([vx, vy, 0.0])

    def _check_alignment(self) -> bool:
        """Kiểm tra điều kiện bàn giao (handoff) dựa trên pose chuẩn của sim."""
        p_cur = p_close(self.q_meas)
        pos_err_world = self.cube_pos - p_cur

        _, R_grip = fk_arm(self.q_meas)
        pos_err_grip = R_grip.T @ pos_err_world

        tilt = gripper_tilt(self.q_meas)
        v_rel = np.linalg.norm(self.cube_vel)

        return (
            abs(pos_err_grip[0]) <= HANDOFF_EX and
            abs(pos_err_grip[1]) <= HANDOFF_EY and
            abs(pos_err_grip[2]) <= HANDOFF_EZ and
            tilt <= HANDOFF_TILT and
            v_rel <= 0.05
        )
