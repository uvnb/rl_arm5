# -*- coding: utf-8 -*-
"""
assarm_rl_env.reward — Hàm reward dense cho giai đoạn APPROACH.

Dùng thông tin đặc quyền của sim (pose thật, contact) cho reward.
Policy chỉ thấy observation (perception có nhiễu).

Xem rl_recommendation_v5_0.md mục 3.5.
"""

import numpy as np


def compute_reward(
    d: float,                    # khoảng cách P_close → khối (m)
    dpsi: float,                 # yaw tương đối đã wrap (rad)
    tilt_deg: float,             # độ nghiêng gripper (°)
    rel_speed: float,            # tốc độ tương đối khối−gripper (m/s)
    jaw_contact: bool,           # ngón mở chạm khối
    action: np.ndarray,          # (4,) action hiện tại
    prev_action: np.ndarray,     # (4,) action trước
    W_NEAR: float = 0.3,
    W_YAW: float = 0.3,
    W_TILT: float = 0.2,
    W_SPEED: float = 0.1,
    W_CONTACT: float = 0.5,
    W_ACTION: float = 0.01,
    w_rate: float = 0.01,
) -> float:
    """
    Tính reward dense cho một step.

    Cấu trúc:
    - `-d`: thưởng khoảng cách (dense, linear, giảm dần xa)
    - `W_NEAR * (1 - tanh(d/0.03))`: bonus khi rất gần (shaping gần target)
    - Phạt yaw, tilt, tốc độ: chỉ tính khi gần (w_near > 0)
    - Phạt contact: ngón mở chạm khối (đẩy khối) rất xấu
    - Phạt action magnitude + rate: giảm lắc giật

    Returns
    -------
    float
        Reward (thường trong [-0.5, +0.3] mỗi step).
    """
    # Hệ số "gần": tăng từ 0 → 1 khi d giảm từ 0.08 → 0
    w_near = np.clip(1.0 - d / 0.08, 0.0, 1.0)

    r = (
        # Distance term — gradient dẫn hướng
        -d
        # Near-target shaping — bonus mạnh khi rất gần
        + W_NEAR * (1.0 - np.tanh(d / 0.03))
        # Yaw penalty — chỉ phạt khi gần (yaw xa không quan trọng)
        - W_YAW * w_near * abs(dpsi) / (np.pi / 4)
        # Tilt penalty — phạt nghiêng quá 15°
        - W_TILT * max(0.0, tilt_deg - 15.0) / 15.0
        # Speed penalty — phạt tốc độ tương đối khi gần
        - W_SPEED * w_near * min(rel_speed / 0.05, 1.0)
        # Contact penalty — ngón mở chạm khối (đẩy khối đi)
        - W_CONTACT * float(jaw_contact)
        # Action regularization
        - W_ACTION * np.sum(action**2)
        # Action rate penalty — giảm lắc giật
        - w_rate * np.sum((action - prev_action)**2)
    )

    return float(r)


def compute_terminal_reward(
    handoff_quality_good: bool,
) -> float:
    """
    Thưởng cuối khi bàn giao.

    handoff_quality_good: True nếu pose thật thỏa mọi ngưỡng.
    Nếu False, script kích hoạt bàn giao nhầm do nhiễu perception.

    Returns
    -------
    float
        +10 nếu tốt, -2 nếu bàn giao nhầm.
    """
    if handoff_quality_good:
        return 10.0
    else:
        return -2.0
