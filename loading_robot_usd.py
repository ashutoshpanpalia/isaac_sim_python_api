from isaacsim import SimulationApp
# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path
import numpy as np
import carb

# Create world
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# Locate assets
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    exit()

# Load UFactory gripper USD into the stage
gripper_usd_path = assets_root_path + "/Isaac/Robots/Ufactory/xarm_gripper/xarm_gripper.usd"
add_reference_to_stage(usd_path=gripper_usd_path, prim_path="/World/UFactoryGripper")
print(gripper_usd_path)

# Reset world
my_world.reset()

print("UFactory gripper loaded successfully!")

# Keep sim running
while simulation_app.is_running():
    my_world.step(render=True)

simulation_app.close()