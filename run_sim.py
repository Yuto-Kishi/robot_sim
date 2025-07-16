# run_sim.py
import mujoco
from mujoco import viewer

# XMLファイルのパス（相対 or 絶対）
xml_path = "XROBOCON_model.xml"

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# MuJoCo GUI起動（閉じるまでこのスクリプトは止まる）
viewer.launch(model, data)

