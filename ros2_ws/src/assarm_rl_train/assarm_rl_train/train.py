# -*- coding: utf-8 -*-
"""
assarm_rl_train.train — Script huấn luyện SAC cho Phase 1 Stage A.

Huấn luyện mô hình SAC gắp khối đứng yên (Level 0-1) đạt handoff_rate >= 90%.
"""

import argparse
import os
import torch
from stable_baselines3 import SAC

from assarm_rl_env.assarm_env import AssarmGraspEnv
from assarm_rl_train.callbacks import HandoffEvalCallback


def main():
    parser = argparse.ArgumentParser(description="Huấn luyện SAC Phase 1 Stage A")
    parser.add_argument("--timesteps", type=int, default=100000, help="Tổng số timesteps huấn luyện")
    parser.add_argument("--level", type=int, default=0, help="Cấp độ spawn (0 hoặc 1 cho Stage A)")
    parser.add_argument("--save_dir", type=str, default="./models/", help="Thư mục lưu mô hình")
    parser.add_argument("--tb_log", type=str, default="./tb_logs/", help="Thư mục Tensorboard")
    args = parser.parse_args()

    # Tối ưu cho CPU i5-1035G4 (4 cores)
    torch.set_num_threads(2)

    print("=" * 70)
    print(f" KHỞI CHẠY HUẤN LUYỆN SAC STAGE A (Level {args.level}, {args.timesteps} timesteps)")
    print("=" * 70)

    # Môi trường train & eval
    env = AssarmGraspEnv(spawn_level=args.level)
    eval_env = AssarmGraspEnv(spawn_level=args.level)

    # Khởi tạo mô hình SAC theo thiết lập v5.0
    model = SAC(
        "MlpPolicy",
        env,
        policy_kwargs=dict(net_arch=[256, 256, 256]),
        gamma=0.98,
        learning_rate=3e-4,
        batch_size=256,
        buffer_size=300_000,
        learning_starts=2000,
        train_freq=1,
        gradient_steps=4,
        ent_coef="auto",
        tensorboard_log=args.tb_log,
        verbose=1,
    )

    # Callback đánh giá
    eval_callback = HandoffEvalCallback(
        eval_env=eval_env,
        eval_freq=5000,
        n_eval_episodes=20,
        save_path=args.save_dir,
        verbose=1,
    )

    # Train
    model.learn(total_timesteps=args.timesteps, callback=eval_callback)

    # Lưu model hoàn tất
    final_path = os.path.join(args.save_dir, "stage_a_final.zip")
    model.save(final_path)
    print(f"\n[DONE] Đã hoàn thành huấn luyện Stage A và lưu tại: {final_path}")


if __name__ == "__main__":
    main()
