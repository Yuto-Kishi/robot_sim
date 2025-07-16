from mujoco import viewer
import mujoco

with open("coin_stage.xml") as f:
    model = mujoco.MjModel.from_xml_string(f.read())
data = mujoco.MjData(model)
viewer.launch(model, data)
