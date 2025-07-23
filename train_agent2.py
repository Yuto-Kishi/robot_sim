import gym
from gym import spaces
import numpy as np
import mujoco
import mujoco.viewer as viewer

class CoinGatherEnv(gym.Env):
    def __init__(self, render_mode=False):
        xml_path = "field_with_robot.xml"
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.render_mode = render_mode
        self.viewer = None

        # Action space: left_wheel, right_wheel, jump actuator
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)

        # Observation space: x, y, z, vx, vy, vz
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32)

    def _get_obs(self):
        xpos = self.data.qpos[:3]
        xvel = self.data.qvel[:3]
        obs = np.concatenate([xpos[:2], xvel[:2], [xpos[2], xvel[2]]])
        return obs.astype(np.float32)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.data.qpos[:] = np.zeros_like(self.data.qpos)
        self.data.qvel[:] = np.zeros_like(self.data.qvel)
        mujoco.mj_forward(self.model, self.data)
        if self.render_mode and self.viewer is None:
            self.viewer = viewer.launch_passive(self.model, self.data)
        obs = self._get_obs()
        return obs, {}

    def step(self, action):
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self.data.ctrl[:3] = action
        for _ in range(5):
            mujoco.mj_step(self.model, self.data)

        obs = self._get_obs()
        reward = self._compute_reward()
        terminated = False
        truncated = False
        return obs, reward, terminated, truncated, {}

    def _compute_reward(self):
        # 簡易な報酬（例：高さ z の最大化）
        z_pos = self.data.qpos[2]
        return z_pos

    def render(self):
        if self.render_mode:
            if self.viewer is None:
                self.viewer = viewer.launch_passive(self.model, self.data)
            self.viewer.sync()

    def close(self):
        if self.viewer:
            self.viewer.close()
            self.viewer = None
