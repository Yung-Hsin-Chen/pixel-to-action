import time

from stable_baselines3 import PPO

from pixel_to_action.envs.husky_maze import HuskyMazeEnv


def main():
    env = HuskyMazeEnv(gui=True)
    model = PPO.load("runs/ppo_husky_goal")

    obs, _ = env.reset()
    for _ in range(5):
        done = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated
            time.sleep(1 / 60)
        obs, _ = env.reset()

    env.close()


if __name__ == "__main__":
    main()
