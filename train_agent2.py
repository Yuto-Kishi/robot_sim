import os
import gymnasium as gym
import mujoco
import mujoco.viewer
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

class CoinGatherEnv(gym.Env):
    def __init__(self, render_mode=False):
        model_path = os.path.abspath("field_with_robot.xml")
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data) if render_mode else None

        # 3 motors: left, right, jump
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)

        obs_size = self.model.nq + self.model.nv
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_size,), dtype=np.float32
        )

    def reset(self, seed=None, options=None):
        mujoco.mj_resetData(self.model, self.data)
        obs = self._get_obs()
        return obs.astype(np.float32), {}

    def step(self, action):
        # Apply motor actions (scale to reasonable torques)
        self.data.ctrl[:3] = action * 1.0
        mujoco.mj_step(self.model, self.data)
        obs = self._get_obs()

        reward = -np.linalg.norm(self.data.qvel)  # optional simple reward
        terminated = False
        truncated = False
        return obs, reward, terminated, truncated, {}

    def _get_obs(self):
        return np.concatenate([self.data.qpos, self.data.qvel]).astype(np.float32)

    def render(self):
        if self.viewer:
            self.viewer.sync()

    def close(self):
        if self.viewer:
            self.viewer.close()

if __name__ == "__main__":
    env = CoinGatherEnv(render_mode=False)
    check_env(env, warn=True)

    model = PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=100000)

    # GUI付きで可視化再生
    test_env = CoinGatherEnv(render_mode=True)
    obs, _ = test_env.reset()
    for _ in range(1000):
        action, _ = model.predict(obs, deterministic=True)
        obs, _, _, _, _ = test_env.step(action)
        test_env.render()
