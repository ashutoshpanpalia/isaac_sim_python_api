# contact_cube.py
import numpy as np
from isaacsim import SimulationApp

# Launch IsaacSim with GUI
simulation_app = SimulationApp({"headless": False})

# Import core API
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.objects.ground_plane import GroundPlane
from isaacsim.core.api.physics_context import PhysicsContext

from isaacsim.sensors.physics import ContactSensor

# -----------------------------------------------------------------------------
# 1) Setup world + physics
# -----------------------------------------------------------------------------

# Initialize physics (must do this before adding contact sensors)
PhysicsContext()

# Ground so the cube will fall and hit it
GroundPlane(
    prim_path="/World/groundPlane",
    size=10,
    z_position=0
)
import omni
from pxr import Sdf, UsdLux
stage = omni.usd.get_context().get_stage()
distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
distantLight.CreateIntensityAttr(300)

# -------------------------------------------------------------------
# 2) Create 2x2 cubes (2x2 tactile grid)
# -------------------------------------------------------------------
GRID_SIZE = 2
CUBE_SIZE = 0.1  # each cube 10cm
CUBE_HEIGHT = 0.05

cubes = []
sensors = []

for i in range(GRID_SIZE):
    for j in range(GRID_SIZE):
        x = (i - 0.5) * CUBE_SIZE  # center cubes around origin
        y = (j - 0.5) * CUBE_SIZE
        z = 0.5  # start above ground to fall

        prim_path = f"/World/Taxel_{i}_{j}"
        cube = DynamicCuboid(
            prim_path=prim_path,
            position=np.array([x, y, z]),
            scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_HEIGHT]),
            color=np.array([0.2 + 0.2*i, 0.3 + 0.1*j, 0.0]),
        )
        cubes.append(cube)

        # Attach contact sensor
        sensor = ContactSensor(
            prim_path=f"{prim_path}/Contact_Sensor",
            name=f"Contact_Sensor_{i}_{j}",
            frequency=120,
            translation=np.array([0.0, 0.0, 0.0]),
            min_threshold=0.0,
            max_threshold=1e6,
            radius=-1,
        )
        sensors.append(sensor)

# -----------------------------------------------------------------------------
# 2) Add a Contact Sensor to that cube
# -----------------------------------------------------------------------------


#Using Python wrapper
# Create the sensor under /World/Cube/Contact_Sensor
sensor = ContactSensor(
    prim_path="/World/Cube/Contact_Sensor",
    name="Contact_Sensor",
    frequency=120,     # sensor update frequency
    translation=np.array([0.0, 0.0, 0.0]),  # sensor is at cube's center
    min_threshold=0.0, # report all contact forces
    max_threshold=1e6,
    radius=-1,         # radius filter deactivated
)

#Using python command

# import omni.kit.commands
# from pxr import Gf
# from isaacsim.sensors.physics import _sensor


# success, _isaac_sensor_prim = omni.kit.commands.execute(
#     "IsaacSensorCreateContactSensor",
#     path="Contact_Sensor",
#     parent="/World/Cube",
#     sensor_period=1,
#     min_threshold=0.0001,
#     max_threshold=100000,
#     translation = Gf.Vec3d(0, 0, 0),
# )


# -----------------------------------------------------------------------------
# 3) Simulation loop
# -----------------------------------------------------------------------------

# Run simulation until GUI is closed
while simulation_app.is_running():

    # Step physics + render
    simulation_app.update()

    # Get current contact sensor frame
    reading = sensor.get_current_frame()

    # Print raw contact force data
    print("Contact Sensor Reading:", reading)


    # _contact_sensor_interface = _sensor.acquire_contact_sensor_interface()
    # raw_data = _contact_sensor_interface.get_contact_sensor_raw_data("/World/Cube/Contact_Sensor")
    # print(str(raw_data))

# -----------------------------------------------------------------------------
# 4) Exit
# -----------------------------------------------------------------------------

simulation_app.close()
