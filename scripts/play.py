import time

import cv2

from pixel_to_action.envs.husky_maze import HuskyMazeEnv


def main():
    env = HuskyMazeEnv(gui=True)
    obs, _ = env.reset()

    for t in range(500):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, _ = env.step(action)

        cv2.imshow("obs", obs)
        cv2.waitKey(1)

        if terminated or truncated:
            obs, _ = env.reset()

        time.sleep(1 / 30)

    env.close()


if __name__ == "__main__":
    main()
