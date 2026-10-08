# -*- coding: utf-8 -*-
"""
assarm_rl_train.callbacks — Callbacks cho quá trình huấn luyện SAC.

Bao gồm:
- EvaluationCallback: Đánh giá định kỳ tỉ lệ bàn giao (handoff_rate)
- CheckpointCallback: Lưu checkpoint mô hình tốt nhất
"""

import os
import numpy as np
from stable_baselines3.common.callbacks import BaseCallback


class HandoffEvalCallback(BaseCallback):
    """
    Callback đánh giá tỉ lệ bàn giao (handoff_rate) định kỳ mỗi eval_freq steps.

    Parameters
    ----------
    eval_env : Gymnasium Env
        Môi trường đánh giá.
    eval_freq : int
        Tần suất đánh giá (mẫu step). Mặc định 5000.
    n_eval_episodes : int
        Số episode mỗi đợt đánh giá. Mặc định 20.
    save_path : str
        Thư mục lưu mô hình tốt nhất.
    """

    def __init__(
        self,
        eval_env,
        eval_freq: int = 5000,
        n_eval_episodes: int = 20,
        save_path: str = "./models/",
        verbose: int = 1,
    ):
        super().__init__(verbose)
        self.eval_env = eval_env
        self.eval_freq = eval_freq
        self.n_eval_episodes = n_eval_episodes
        self.save_path = save_path
        self.best_handoff_rate = 0.0

        os.makedirs(self.save_path, exist_ok=True)

    def _on_step(self) -> bool:
        if self.n_calls % self.eval_freq == 0:
            handoff_count = 0
            total_rewards = []

            for _ in range(self.n_eval_episodes):
                obs, _ = self.eval_env.reset()
                done = False
                ep_reward = 0.0

                while not done:
                    action, _ = self.model.predict(obs, deterministic=True)
                    obs, reward, terminated, truncated, info = self.eval_env.step(action)
                    ep_reward += reward
                    done = terminated or truncated

                    if info.get("handoff_ok", False):
                        handoff_count += 1

                total_rewards.append(ep_reward)

            handoff_rate = (handoff_count / self.n_eval_episodes) * 100.0
            mean_reward = float(np.mean(total_rewards))

            if self.verbose > 0:
                print(f"\n[Eval @ Step {self.num_timesteps}] Handoff Rate: {handoff_rate:.1f}% ({handoff_count}/{self.n_eval_episodes}), Mean Reward: {mean_reward:.2f}")

            self.logger.record("eval/handoff_rate", handoff_rate)
            self.logger.record("eval/mean_reward", mean_reward)

            # Luôn lưu checkpoint mới nhất cùng replay buffer
            latest_model_path = os.path.join(self.save_path, "latest_model.zip")
            latest_buffer_path = os.path.join(self.save_path, "latest_replay_buffer.pkl")
            self.model.save(latest_model_path)
            try:
                self.model.save_replay_buffer(latest_buffer_path)
            except Exception as e:
                if self.verbose > 0:
                    print(f"--> Warning: Could not save replay buffer: {e}")

            if handoff_rate > self.best_handoff_rate:
                self.best_handoff_rate = handoff_rate
                best_model_path = os.path.join(self.save_path, "best_model.zip")
                best_buffer_path = os.path.join(self.save_path, "best_replay_buffer.pkl")
                self.model.save(best_model_path)
                try:
                    self.model.save_replay_buffer(best_buffer_path)
                except Exception:
                    pass
                if self.verbose > 0:
                    print(f"--> Saved new best model & buffer to {best_model_path} (handoff_rate = {handoff_rate:.1f}%)")

        return True
