import mujoco
import mujoco.viewer
import numpy as np
import time

# モデルファイルのパス
MODEL_PATH = "robot_only.xml"

# MuJoCoモデルとデータ読み込み
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# GUIの表示（mjpython不要）
try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("MuJoCo viewer launched. Press ESC to exit.")
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(0.01)
except Exception as e:
    print(f"MuJoCo viewer failed to launch: {e}")
