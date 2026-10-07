# -*- coding: utf-8 -*-
"""
assarm_common — Cấu hình hằng số dùng chung giữa sim và robot thật.

Mọi con số ở đây là giá trị khởi đầu, sẽ hiệu chỉnh bằng đo đạc thật (Phase R).
Xem rl_recommendation_v5_0.md mục 3.x cho nguồn gốc từng giá trị.
"""

import numpy as np

# ===========================================================================
# Giới hạn khớp (từ URDF, rad)
# ===========================================================================
Q_LO_ARM = np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2])
Q_HI_ARM = np.array([+np.pi / 2, +np.pi / 2, +np.pi / 2, +np.pi / 2])

JOINT_NAMES_ARM = [
    "Revolute_Joint_1",
    "Revolute_Joint_2",
    "Revolute_Joint_3",
    "Revolute_Joint_4",
]
JOINT_NAME_GRIPPER = "Revolute_Joint_Active"

# ===========================================================================
# Gripper
# ===========================================================================
Q_STAR = 0.896       # rad — góc đóng ôm khít khối 5 cm (từ FK trên URDF)
EPS_GRIP = 0.1       # rad — dư để có lực kẹp
Q_OPEN = 0.0         # rad — gripper mở hoàn toàn
Q_GRIP_MAX = np.pi / 2  # rad — giới hạn URDF

# Offset tâm gắp khi đóng (khung gripper, m)
# P_close = gc_open + R_gripper @ GRIP_CLOSE_OFFSET
GRIP_CLOSE_OFFSET = np.array([0.0, 0.0, -0.0312])

# ===========================================================================
# Cửa sổ VERIFY (rad)
# ===========================================================================
VERIFY_WINDOW_LO = 0.82  # rad
VERIFY_WINDOW_HI = 0.97  # rad
MAX_VERIFY_RETRIES = 2

# ===========================================================================
# Ngưỡng bàn giao (handoff)
# ===========================================================================
HANDOFF_EX = 0.006          # m — |e_x| trục đóng
HANDOFF_EY = 0.010          # m — |e_y| dọc ngón
HANDOFF_EZ = 0.006          # m — |e_z| trục gripper
HANDOFF_DPSI = np.radians(10)   # rad — |Δψ|
HANDOFF_TILT = np.radians(15)   # rad — độ nghiêng gripper
HANDOFF_REL_SPEED = 0.02        # m/s — tốc độ tương đối
HANDOFF_CONSEC_STEPS = 3        # số step liên tiếp

# ===========================================================================
# Điều khiển
# ===========================================================================
DT_CTRL = 0.1          # s — chu kỳ điều khiển 10 Hz
DT_PHYS = 0.002        # s — max_step_size Gazebo
SIM_SUBSTEPS = int(DT_CTRL / DT_PHYS)  # 50
CMD_SUBSTEPS = 5        # 5 lệnh con 20 ms (khớp firmware ESP32)

MAX_DELTA = 0.08        # rad/step — bước tối đa mỗi action
EPS_ARM = 0.25          # rad — giới hạn |target − q_meas|

# ===========================================================================
# Chuẩn hóa observation (thủ công, range cố định)
# ===========================================================================
NORM_JOINT = np.pi / 2  # chia giới hạn khớp
NORM_VEL = 2.0          # rad/s — clip ±1
NORM_POS = 0.3          # m
NORM_OBJ_VEL = 0.1      # m/s — clip ±1
NORM_ERR = 0.1          # m
NORM_TILT = 0.5         # rad

# ===========================================================================
# Reward hyperparameters
# ===========================================================================
W_NEAR = 0.3            # hệ số shaping gần
W_YAW = 0.3             # hệ số phạt yaw
W_TILT = 0.2            # hệ số phạt nghiêng
W_SPEED = 0.1           # hệ số phạt tốc độ tương đối
W_CONTACT = 0.5         # hệ số phạt contact ngón–khối
W_ACTION = 0.01         # hệ số phạt action
W_RATE = 0.01           # hệ số phạt action rate (tăng dần tới 0.03–0.05)
REWARD_HANDOFF_GOOD = 10.0   # thưởng bàn giao tốt
REWARD_HANDOFF_BAD = -2.0    # phạt bàn giao nhầm

# ===========================================================================
# Episode
# ===========================================================================
MAX_STEPS_STAGE_A = 60
MAX_STEPS_STAGE_B = 80

# ===========================================================================
# Domain Randomization — khoảng ban đầu
# ===========================================================================
DR_CUBE_MASS = (0.07, 0.13)          # kg (0.1 ± 30%)
DR_CUBE_SIZE = (0.047, 0.053)        # m (0.05 ± 3mm mỗi cạnh)
DR_FRICTION_GRIPPER = (0.5, 2.0)     # μ
DR_EFFORT_SCALE = (0.8, 1.2)         # ± 20%
DR_SERVO_GAIN_SCALE = (0.7, 1.3)     # ± 30%
DR_PENDULUM_LENGTH_SCALE = (0.9, 1.1)  # ± 10%
DR_PENDULUM_DAMPING_RANGE = (0.001, 0.01)  # log-uniform quanh số đo

# ===========================================================================
# Spawn levels (bán kính ngang m, góc quét J1 rad, độ cao khối m,
#               yaw rad, biên độ đung đưa m)
# ===========================================================================
# ===========================================================================
# Spawn levels (bán kính ngang m, góc quét J1 rad, độ cao khối m,
#               yaw rad, biên độ đung đưa m)
# Cập nhật v5.0: Phase 1 (Level 0-1) đứng yên (swing=0.0), Phase 2 (Level 2-3) đung đưa nhẹ (<1cm)
# ===========================================================================
SPAWN_LEVELS = {
    0: dict(r=(0.10, 0.14), theta=np.radians(15), z=(0.19, 0.25),
            yaw=np.radians(15), swing=0.0),
    1: dict(r=(0.09, 0.16), theta=np.radians(20), z=(0.19, 0.25),
            yaw=np.radians(20), swing=0.0),
    2: dict(r=(0.08, 0.18), theta=np.radians(35), z=(0.18, 0.26),
            yaw=np.radians(25), swing=0.005),
    3: dict(r=(0.08, 0.19), theta=np.radians(60), z=(0.18, 0.26),
            yaw=np.radians(45), swing=0.008),
}

# ===========================================================================
# Thông số hiệu chỉnh phần cứng Phase R (Đã đo đạc từ flash/code Arduino)
# ===========================================================================
# Joint 1: TD8120MG
SERVO_J1_PULSE_MIN = 490   # us (0 deg)
SERVO_J1_PULSE_MAX = 2520  # us (180 deg)
SERVO_J1_ADC0 = 2533       # mV (0 deg)
SERVO_J1_ADCMID = 1616     # mV (90 deg)
SERVO_J1_ADC180 = 705      # mV (180 deg)
SERVO_J1_RATIO = 1.0       # Noi thang GPIO34

# Joint 2: RDS3120MG
SERVO_J2_PULSE_MIN = 510   # us (0 deg)
SERVO_J2_PULSE_MAX = 2440  # us (180 deg)
SERVO_J2_ADC0 = 756        # mV wiper (0 deg)
SERVO_J2_ADCMID = 3220     # mV wiper (90 deg)
SERVO_J2_ADC180 = 5790     # mV wiper (180 deg)
SERVO_J2_RATIO = 2.0       # Cau chia ap 47k+47k

# Joint 3, 4, Gripper: MG995
SERVO_MG995_PULSE_MIN = 500  # us (0 deg)
SERVO_MG995_PULSE_MAX = 2500 # us (180 deg)
SERVO_MG995_ADC0 = 494       # mV wiper (0 deg)
SERVO_MG995_ADCMID = 3332    # mV wiper (90 deg)
SERVO_MG995_ADC180 = 6186    # mV wiper (180 deg)
SERVO_MG995_RATIO = 2.0      # Cau chia ap 47k+47k

