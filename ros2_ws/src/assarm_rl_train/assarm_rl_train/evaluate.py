# -*- coding: utf-8 -*-
"""
assarm_rl_train.evaluate — Script đánh giá mô hình SAC sau khi huấn luyện.

Đánh giá 100/200 episode theo chuẩn mực Phase 4:
- handoff_rate (mục tiêu >= 90%)
- overall_success (mục tiêu >= 85%)
- false_handoff_rate (mục tiêu < 5%)
"""

import argparse
import numpy as np
from stable_baselines3 import SAC

from assarm_common.grasp_script import GraspScript, GraspState
from assarm_rl_env.assarm_env import AssarmGraspEnv


def evaluate_policy(model_path: str, n_episodes: int = 100, level: int = 0):
    """
    Đánh giá mô hình SAC gắp khối.

    Parameters
    ----------
    model_path : str
        Đường dẫn tới file .zip mô hình SAC.
    n_episodes : int
        Số episode đánh giá (mặc định 100).
    level : int
        Level spawn khối (0..3).
    """
    print("=" * 70)
    print(f" ĐÁNH GIÁ MÔ HÌNH SAC ({model_path}) — Level {level}, {n_episodes} Episodes")
    print("=" * 70)

    env = AssarmGraspEnv(spawn_level=level)
    model = SAC.load(model_path, env=env)

    handoff_count = 0
    grasp_success_count = 0
    false_handoff_count = 0
    ep_rewards = []

    for ep in range(n_episodes):
        obs, info = env.reset(seed=42 + ep)
        script = GraspScript(dt=0.1, stall_threshold=0.12)
        script.reset(env.sim.q_meas)

        done = False
        ep_reward = 0.0
        handoff_trig = False
        in_script_phase = False

        for step in range(env.max_steps):
            if in_script_phase:
                # Chạy script gắp-giữ sau bàn giao
                grip_fb = 0.896 if script._gripper_target > 0.896 else script._gripper_target
                cmd = script.step(env.sim.q_meas, grip_fb)
                obs, reward, terminated, truncated, info = env.step((cmd.arm_target - env.sim.q_meas) / 0.08)
                if script.is_terminal:
                    break
            else:
                # RL điều khiển tiếp cận
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = env.step(action)
                ep_reward += reward

                if info.get("handoff_ok", False):
                    handoff_trig = True
                    in_script_phase = True
                    script.trigger_handoff(env.sim.q_meas)

                if terminated or truncated:
                    break

        if handoff_trig:
            handoff_count += 1
            if not info.get("handoff_ok", True):
                false_handoff_count += 1

        if script.state in (GraspState.HOLD, GraspState.DONE, GraspState.RELEASE) or (handoff_trig and not script.is_terminal):
            grasp_success_count += 1

        ep_rewards.append(ep_reward)

    handoff_rate = (handoff_count / n_episodes) * 100.0
    overall_success = (grasp_success_count / n_episodes) * 100.0
    false_handoff_rate = (false_handoff_count / max(handoff_count, 1)) * 100.0

    print("\n" + "=" * 70)
    print(" KẾT QUẢ ĐÁNH GIÁ MÔ HÌNH SAC")
    print("=" * 70)
    print(f"  Handoff Rate (Tỉ lệ bàn giao): {handoff_rate:.1f}% ({handoff_count}/{n_episodes})")
    print(f"  Overall Success (Gắp thành công): {overall_success:.1f}% ({grasp_success_count}/{n_episodes})")
    print(f"  False Handoff Rate (Bàn giao nhầm): {false_handoff_rate:.1f}% ({false_handoff_count}/{max(handoff_count, 1)})")
    print(f"  Mean Episode Reward: {np.mean(ep_rewards):.2f} ± {np.std(ep_rewards):.2f}")
    print("=" * 70)

    return handoff_rate, overall_success, false_handoff_rate


def main():
    parser = argparse.ArgumentParser(description="Đánh giá mô hình SAC")
    parser.add_argument("--model", type=str, required=True, help="Đường dẫn file mô hình .zip")
    parser.add_argument("--episodes", type=int, default=100, help="Số episode đánh giá")
    parser.add_argument("--level", type=int, default=0, help="Level spawn (0..3)")
    args = parser.parse_args()

    evaluate_policy(args.model, n_episodes=args.episodes, level=args.level)


if __name__ == "__main__":
    main()
