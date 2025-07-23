import mujoco
from mujoco import viewer

# モデル読み込み
model = mujoco.MjModel.from_xml_path("field_with_real_robot.xml")
data = mujoco.MjData(model)

# GUI起動
viewer.launch(model, data)
