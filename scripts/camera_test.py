import random
import time

import cv2
import numpy as np
import pybullet as p
import pybullet_data

W, H = 84, 84

# Husky has 4 wheel joints: 2, 3, 4, 5 in the default URDF
wheel_joints = [2, 3, 4, 5]


def main():
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)

    p.loadURDF("plane.urdf")
    husky = p.loadURDF("husky/husky.urdf", [0, 0, 0.1])

    def get_camera_image():
        pos, orn = p.getBasePositionAndOrientation(husky)
        rot = p.getMatrixFromQuaternion(orn)

        # Robot forward direction (x-axis of base frame)
        forward = np.array([rot[0], rot[3], rot[6]])
        cam_pos = np.array([pos[0], pos[1], pos[2] + 0.3])
        target = cam_pos + forward * 1.0

        view = p.computeViewMatrix(cam_pos.tolist(), target.tolist(), [0, 0, 1])
        proj = p.computeProjectionMatrixFOV(fov=60, aspect=W / H, nearVal=0.1, farVal=20.0)

        _, _, rgba, _, _ = p.getCameraImage(W, H, view, proj, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        rgb = np.reshape(rgba, (H, W, 4))[:, :, :3].astype(np.uint8)
        return rgb

    def set_action(action: int):
        # 0=left, 1=forward, 2=right
        if action == 0:
            left, right = 4.0, 8.0
        elif action == 2:
            left, right = 8.0, 4.0
        else:
            left, right = 8.0, 8.0

        # left wheels: joints 2,4 ; right wheels: joints 3,5 (typical husky)
        p.setJointMotorControl2(husky, 2, p.VELOCITY_CONTROL, targetVelocity=left, force=200)
        p.setJointMotorControl2(husky, 4, p.VELOCITY_CONTROL, targetVelocity=left, force=200)
        p.setJointMotorControl2(husky, 3, p.VELOCITY_CONTROL, targetVelocity=right, force=200)
        p.setJointMotorControl2(husky, 5, p.VELOCITY_CONTROL, targetVelocity=right, force=200)

    step = 0
    while True:
        if step % 30 == 0:
            set_action(random.randint(0, 2))

        img = get_camera_image()
        cv2.imshow("husky camera", img)
        cv2.waitKey(1)

        p.stepSimulation()
        time.sleep(1 / 240)
        step += 1


if __name__ == "__main__":
    main()
