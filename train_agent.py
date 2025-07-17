from stable_baselines3 import PPO
from robot_env import CoinGatherEnv

env = CoinGatherEnv(render_mode=False)
model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./tb")
model.learn(total_timesteps=100_000)

# テスト実行
test_env = CoinGatherEnv(render_mode=True)
obs = test_env.reset()
for _ in range(1000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, _ = test_env.step(action)
    test_env.render()
from mujoco import viewer

# 簡易的に学習済みロボットを表示
env = CoinGatherEnv(render_mode=True)
obs = env.reset()
viewer.launch(env.model, env.data)
