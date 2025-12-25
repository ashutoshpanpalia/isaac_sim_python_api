# # contact_2x2_heatmap.py
# import numpy as np
# import matplotlib.pyplot as plt
# from isaacsim import SimulationApp

# # Launch IsaacSim
# simulation_app = SimulationApp({"headless": False})

# # Core API
# from isaacsim.core.api.objects import DynamicCuboid
# from isaacsim.core.api.objects.ground_plane import GroundPlane
# from isaacsim.core.api.physics_context import PhysicsContext
# from isaacsim.sensors.physics import ContactSensor

# # ----------------------------
# # 1) Setup world + physics
# # ----------------------------
# PhysicsContext()  # initialize physics

# # Ground
# GroundPlane(prim_path="/World/groundPlane", size=10, z_position=0)

# import omni
# from pxr import Sdf, UsdLux
# stage = omni.usd.get_context().get_stage()
# distantLight = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
# distantLight.CreateIntensityAttr(300)

# # ----------------------------
# # 2) Create 2x2 cubes with contact sensors
# # ----------------------------
# GRID_SIZE = 2
# CUBE_SIZE = 0.1
# CUBE_HEIGHT = 0.05

# cubes = []
# sensors = []

# for i in range(GRID_SIZE):
#     for j in range(GRID_SIZE):
#         x = (i - 0.5) * CUBE_SIZE
#         y = (j - 0.5) * CUBE_SIZE
#         z = 0.5  # start above ground

#         prim_path = f"/World/Taxel_{i}_{j}"
#         cube = DynamicCuboid(
#             prim_path=prim_path,
#             position=np.array([x, y, z]),
#             scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_HEIGHT]),
#             color=np.array([0.2 + 0.2*i, 0.3 + 0.1*j, 0.0]),
#         )
#         cubes.append(cube)

#         # Contact sensor
#         sensor = ContactSensor(
#             prim_path=f"{prim_path}/Contact_Sensor",
#             name=f"Contact_Sensor_{i}_{j}",
#             frequency=120,
#             translation=np.array([0.0, 0.0, 0.0]),
#             min_threshold=0.0,
#             max_threshold=1e6,
#             radius=-1,
#         )
#         sensors.append(sensor)


# # ----------------------------
# # 4) Setup matplotlib heatmap
# # ----------------------------
# plt.ion()
# fig, ax = plt.subplots()
# force_grid = np.zeros((GRID_SIZE, GRID_SIZE))
# heatmap = ax.imshow(force_grid, cmap="Reds", vmin=0, vmax=1)
# plt.colorbar(heatmap)
# ax.set_title("Tactile Force Heatmap")
# ax.set_xticks(np.arange(GRID_SIZE))
# ax.set_yticks(np.arange(GRID_SIZE))
# ax.set_xticklabels(range(GRID_SIZE))
# ax.set_yticklabels(range(GRID_SIZE))

# # ----------------------------
# # 5) Simulation loop
# # ----------------------------
# while simulation_app.is_running():
#     simulation_app.update()

#     # Read sensor data
#     for i in range(GRID_SIZE):
#         for j in range(GRID_SIZE):
#             idx = i * GRID_SIZE + j
#             reading = sensors[idx].get_current_frame()
#             force_grid[i, j] = reading.get("force", 0.0) if reading.get("in_contact", False) else 0.0

#     # Update heatmap
#     heatmap.set_data(force_grid)
#     heatmap.autoscale()
#     plt.draw()
#     plt.pause(0.001)

# # ----------------------------
# # 6) Exit
# # ----------------------------
# simulation_app.close()


############# RIGID PRIM #####################

import numpy as np
from isaacsim import SimulationApp

# Start simulation
simulation_app = SimulationApp({"headless": False})



from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import RigidPrim
import matplotlib.pyplot as plt

# ------------------------------
# World setup
# ------------------------------
world = World(stage_units_in_meters=1.0, backend="numpy")
world.scene.add_default_ground_plane()
world._physics_context.set_gravity(-9.81)

# Physics material for all cubes
material = PhysicsMaterial(
    prim_path="/World/PhysicsMaterials",
    static_friction=0.5,
    dynamic_friction=0.5,
)

# 2x2 cube positions (spawn from height)
cube_positions = [
    [-0.5, -0.5, 3.0],
    [0.5, -0.5, 3.0],
    [-0.5, 0.5, 3.0],
    [0.5, 0.5, 5.0],
]

# Spawn cubes
for i, pos in enumerate(cube_positions):
    cube = DynamicCuboid(
        prim_path=f"/World/Cube_{i+1}",
        name=f"cube_{i+1}",
        size=0.50,
        color=np.array([0.5, 0, 0]),
        mass=5.0,
        position=pos,
    )
    cube.apply_physics_material(material)

# ------------------------------
# Create a RigidPrim view to batch-track contacts
# ------------------------------
cube_view = RigidPrim(
    prim_paths_expr="/World/Cube_*",
    name="cube_view",
    positions=np.array(cube_positions),
    contact_filter_prim_paths_expr=[
        "/World/defaultGroundPlane/GroundPlane/CollisionPlane"
    ],
    max_contact_count=4 * 10,  # 4 cubes x 10 possible contacts
)

world.scene.add(cube_view)
world.reset(soft=False)

# ------------------------------
# Setup matplotlib heatmap
# ------------------------------
plt.ion()  # interactive mode
fig, ax = plt.subplots()
heatmap = ax.imshow(np.zeros((2, 2)), cmap="hot", interpolation="nearest")
cbar = plt.colorbar(heatmap)
cbar.set_label("Normal Force (N)")
ax.set_title("Normal Force Heatmap (2x2 cubes)")

# ------------------------------
# Simulation loop
# ------------------------------
while simulation_app.is_running():
    world.step(render=True)

    # Get contact data
    (
        contact_forces,
        contact_points,
        contact_normals,
        distances,
        pair_contacts_count,
        pair_contacts_start_indices,
    ) = cube_view.get_contact_force_data(dt=1 / 60)

    # Initialize arrays
    force_aggregate = np.zeros((len(cube_positions), 1, 3))
    contact_position = np.zeros((len(cube_positions), 1, 3))

    # Aggregate contact forces and positions
    for i in range(pair_contacts_count.shape[0]):  # each cube
        for j in range(pair_contacts_count.shape[1]):  # each filter
            start_idx = pair_contacts_start_indices[i, j]
            count = pair_contacts_count[i, j]
            if count > 0:
                pair_forces = contact_forces[start_idx : start_idx + count]
                pair_normals = contact_normals[start_idx : start_idx + count]
                force_aggregate[i, j] = np.sum(pair_forces * pair_normals, axis=0)
                contact_position[i, j] = np.sum(contact_points[start_idx : start_idx + count], axis=0) / count

    # # Print forces and contact positions
    # print("====== Contact Forces (2x2 cube grid) ======")
    # for idx, f in enumerate(force_aggregate[:, 0, :]):
    #     print(f"Cube {idx+1}: Force {f}, Contact Point {contact_position[idx,0,:]}")

    # Update heatmap for normal forces
    Fz_grid = force_aggregate[:, 0, 2].reshape(2, 2)
    heatmap.set_data(Fz_grid)
    heatmap.set_clim(vmin=np.min(Fz_grid), vmax=np.max(Fz_grid))
    plt.pause(0.01)

# Close simulation
simulation_app.close()
plt.ioff()
plt.show()