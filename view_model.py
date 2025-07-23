import mujoco
from mujoco import viewer

# XMLファイルを読み込む
model = mujoco.MjModel.from_xml_path("field_with_real_robot.xml")
data = mujoco.MjData(model)

# GUIで表示
viewer.launch(model, data)
