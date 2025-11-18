import sys
import numpy as np
import genesis as gs
from scenes import create_scene_6blocks, create_scene_stacked
from symbolic_abstraction import generate_pddl
import pyperplan
import subprocess
from pathlib import Path



# Ensure Genesis is initialized before building scenes
if len(sys.argv) > 1 and sys.argv[1] == "gpu":
    gs.init(backend=gs.gpu, logging_level='Warning', logger_verbose_time=False)
else:
    gs.init(backend=gs.cpu, logging_level='Warning', logger_verbose_time=False)

# build the scene using the factory
# scene, franka, BlocksState = create_scene_6blocks()
scene, franka, BlocksState = create_scene_stacked()

# Symbolically abstract scene to formulate pddl problem (generates .pddl file after call)
generate_pddl(scene, franka, BlocksState)

# Check if pddl was properly generated, otherwise, throw an error
pddl_problem = Path("problem.pddl")
if not pddl_problem.exists():
    raise FileNotFoundError(f"The file {pddl_problem} does not exist.")
else:
    pddl_domain = Path("pyperplan/benchmarks/blocks/domain.pddl")
    # Save actions to .soln file
    subprocess.run([
        "pyperplan", str(pddl_domain), str(pddl_problem)],
        check=True
    )
    # Rename file
    Path("problem.pddl.soln").rename("actions.soln")



# set control gains
# Note: the following values are tuned for achieving best behavior with Franka
# Typically, each new robot would have a different set of parameters.
# Sometimes high-quality URDF or XML file would also provide this and will be parsed.

franka.set_dofs_kp(
    np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
)
franka.set_dofs_kv(
    np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
)
franka.set_dofs_force_range(
    np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
    np.array([87, 87, 87, 87, 12, 12, 12, 100, 100]),
)

# move to a fixed pre-grasp pose
qpos = franka.inverse_kinematics(
    link=franka.get_link("hand"),
    pos=np.array([0.65, 0.0, 0.25]),
    quat=np.array([0, 1, 0, 0]),
)
# gripper open pos
qpos[-2:] = 0.04
path = franka.plan_path(
    qpos_goal=qpos,
    num_waypoints=200,  # 2s duration
)
# execute the planned path
for waypoint in path:
    franka.control_dofs_position(waypoint)
    scene.step()


# CODE FROM PROJECT 0
# allow robot to reach the last waypoint
for i in range(100):
    scene.step()

# get the end-effector link
end_effector = franka.get_link("hand")
motors_dof = np.arange(7)
fingers_dof = np.arange(7, 9)

# reach
qpos = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.65, 0.0, 0.130]),
    quat=np.array([0, 1, 0, 0]),
)
franka.control_dofs_position(qpos[:-2], motors_dof)
for i in range(100):
    scene.step()


# grasp
franka.control_dofs_position(qpos[:-2], motors_dof)
franka.control_dofs_force(np.array([-0.5, -0.5]), fingers_dof)

for i in range(100):
    scene.step()

# lift
qpos = franka.inverse_kinematics(
    link=end_effector,
    pos=np.array([0.65, 0.0, 0.28]),
    quat=np.array([0, 1, 0, 0]),
)
franka.control_dofs_position(qpos[:-2], motors_dof)
for i in range(200):
    scene.step()