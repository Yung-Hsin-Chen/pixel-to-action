from __future__ import annotations

import gymnasium as gym
import numpy as np
import pybullet as p
import pybullet_data
from gymnasium import spaces


class HuskyMazeEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self, gui: bool = False, obs_size: int = 84, max_steps: int = 500):
        super().__init__()
        self.gui = gui
        self.obs_size = obs_size
        self.max_steps = max_steps

        self.action_space = spaces.Discrete(3)  # left, forward, right
        self.observation_space = spaces.Box(
            low=0, high=255, shape=(obs_size, obs_size, 3), dtype=np.uint8
        )

        self._cid = None
        self._husky = None
        self._step_count = 0

    def _connect(self):
        if self._cid is None:
            self._cid = p.connect(p.GUI if self.gui else p.DIRECT)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())

    def _reset_world(self):
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        self._husky = p.loadURDF("husky/husky.urdf", [0, 0, 0.1])

        # ----- WALLS (visible + collidable) -----
        wall_half = [0.1, 2.0, 0.3]  # (x, y, z) half extents
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=wall_half)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=wall_half, rgbaColor=[0.7, 0.7, 0.7, 1])

        # Place wall so it sits on the ground: z = half height
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=[1.0, 0.0, wall_half[2]],
        )

        # A second wall to make it obvious
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=[0.0, 1.5, wall_half[2]],
        )

    def _set_action(self, action: int):
        if action == 0:
            left, right = 4.0, 8.0
        elif action == 2:
            left, right = 8.0, 4.0
        else:
            left, right = 8.0, 8.0

        p.setJointMotorControl2(self._husky, 2, p.VELOCITY_CONTROL, targetVelocity=left, force=200)
        p.setJointMotorControl2(self._husky, 4, p.VELOCITY_CONTROL, targetVelocity=left, force=200)
        p.setJointMotorControl2(self._husky, 3, p.VELOCITY_CONTROL, targetVelocity=right, force=200)
        p.setJointMotorControl2(self._husky, 5, p.VELOCITY_CONTROL, targetVelocity=right, force=200)

    def _get_obs(self) -> np.ndarray:
        W = H = self.obs_size
        pos, orn = p.getBasePositionAndOrientation(self._husky)
        rot = p.getMatrixFromQuaternion(orn)
        forward = np.array([rot[0], rot[3], rot[6]])

        cam_pos = np.array([pos[0], pos[1], pos[2] + 0.3])
        target = cam_pos + forward * 1.0

        view = p.computeViewMatrix(cam_pos.tolist(), target.tolist(), [0, 0, 1])
        proj = p.computeProjectionMatrixFOV(60, W / H, 0.1, 20.0)

        _, _, rgba, _, _ = p.getCameraImage(W, H, view, proj, renderer=p.ER_BULLET_HARDWARE_OPENGL)
        rgb = np.reshape(rgba, (H, W, 4))[:, :, :3].astype(np.uint8)
        return rgb

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._connect()
        self._reset_world()
        self._step_count = 0
        obs = self._get_obs()
        info = {}
        return obs, info

    def step(self, action: int):
        self._set_action(int(action))

        # Simulate a few physics steps per RL step
        for _ in range(8):
            p.stepSimulation()

        obs = self._get_obs()

        reward = 0.0  # placeholder for now
        terminated = False
        truncated = self._step_count >= self.max_steps

        self._step_count += 1
        info = {}
        return obs, float(reward), terminated, truncated, info

    def close(self):
        if self._cid is not None:
            p.disconnect(self._cid)
            self._cid = None
