import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv

from pixel_to_action.envs.husky_maze import HuskyMazeEnv


def main():
    def make_env():
        return HuskyMazeEnv(gui=False, max_steps=500)

    env = DummyVecEnv([make_env])

    device = "mps" if torch.backends.mps.is_available() else "cpu"
    print("Using device:", device)

    model = PPO(
        policy="CnnPolicy",
        env=env,
        verbose=1,
        device=device,
        n_steps=1024,
        batch_size=64,
    )
    model.learn(total_timesteps=200_000)
    model.save("runs/ppo_husky_goal")


if __name__ == "__main__":
    main()
