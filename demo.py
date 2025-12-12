import sys
import numpy as np
import genesis as gs
from scenes import create_scene_6blocks, create_scene_stacked, create_scene_special_1, create_scene_special_2
from symbolic_abstraction import generate_pddl, generate_pddl_special
import pyperplan
import subprocess
from pathlib import Path
import motion_primitives as motionp
from time import sleep


# Ensure Genesis is initialized before building scenes
if len(sys.argv) > 1 and sys.argv[1] == "gpu":
    gs.init(backend=gs.gpu, logging_level='Warning', logger_verbose_time=False)
else:
    gs.init(backend=gs.cpu, logging_level='Warning', logger_verbose_time=False)


# Asking user for input for desired goal
print("\nPlease enter the corresponding number for your desired goal:\n")
print("Goal #1: Build the Two Towers")
print("Goal #2: 5-Block Tower")
print("Goal #3: Tallest Tower Challenge (Bonus)")
print("Goal #4: Special Structure #1 (167)")
print("Goal #5: Special Structure #2 (61)\n")
goal_num = int(input("Desired goal number: "))

# If they choose either of the first two goals, ask desired starting scene
scene_num = 0
if goal_num == 1 or goal_num == 2:
   print("\nPlease enter the desired starting scene number:\n")
   print("Starting Scene #1: 6 Blocks")
   print("Starting Scene #2: Stacked")
   scene_num = int(input("\nDesired starting scene number: "))

# Create the desired scene using the factory
# Continually ask for values for goal or scene until enter appropriate values 
valid_goal = False
valid_scene = False
while not valid_goal:
    if goal_num == 1 or goal_num == 2:
        while not valid_scene:
            if scene_num == 1:
                scene, franka, BlocksState = create_scene_6blocks()
                valid_scene = True
            elif scene_num == 2:
                scene, franka, BlocksState = create_scene_stacked()
                valid_scene = True
            else:
                scene_num = int(input("Please enter a valid scene number (1 or 2): "))
        valid_goal = True
    elif goal_num == 4:
        scene, franka, BlocksState, SlotsState = create_scene_special_1()
        valid_goal = True
    elif goal_num == 3 or goal_num == 5:
        scene, franka, BlocksState, SlotsState = create_scene_special_2() 
        valid_goal = True
    else:
        goal_num = int(input("Please enter a valid goal number (1-5): "))

motion = motionp.MotionPrimitives(franka, scene, BlocksState)

# Symbolically abstract scene to formulate pddl problem (generates .pddl file after call)
if goal_num == 1 or goal_num == 2 or goal_num == 3:
    generate_pddl(scene, franka, BlocksState, goal_num)
else:
    generate_pddl_special(scene, franka, BlocksState, SlotsState, goal_num)

# Check if pddl was properly generated, otherwise, throw an error
pddl_problem = Path("problem.pddl")
if not pddl_problem.exists():
    raise FileNotFoundError(f"The file {pddl_problem} does not exist.")
else:
    if goal_num == 1 or goal_num == 2 or goal_num == 3:
        pddl_domain = Path("domain.pddl")
        # Run pyperplan with bfs
        subprocess.run([
            "pyperplan", str(pddl_domain), str(pddl_problem)],
            check=True
        )
    else:
        pddl_domain = Path("custom_domain.pddl")
        # Run pyperplan with greedy best first search rather than bfs
        subprocess.run([
            "pyperplan", "-s", "gbf", str(pddl_domain), str(pddl_problem)],
            check=True
        )
    # Save actions to .soln file & rename it 
    Path("problem.pddl.soln").rename("actions.soln")

franka.set_dofs_kp(
    np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
)
franka.set_dofs_kv(
    np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
)
franka.set_dofs_force_range(
    np.array([-60, -60, -60, -60, -10, -10, -10, -100, -100]),
    np.array([60, 60, 60, 60, 10, 10, 10, 100, 100]),
)

# No re-planning
# motion.runSolution('actions.soln')

# With re-planning
finished = False
while not finished:
    with open("actions.soln", "r") as f:
        content = f.read()
        if not content:
            print("The goal has been reached!")
            break
        else:
            print("Re-ground predicates and re-planning")
            finished = motion.runSolutionStep("actions.soln")
            motion.scene.step(50)
            # Symbolically abstract scene to formulate pddl problem (generates .pddl file after call)
            if goal_num == 1 or goal_num == 2 or goal_num == 3:
                generate_pddl(scene, franka, BlocksState, goal_num)
            else:
                generate_pddl_special(scene, franka, BlocksState, SlotsState, goal_num)

            # Check if pddl was properly generated, otherwise, throw an error
            pddl_problem = Path("problem.pddl")
            if not pddl_problem.exists():
                raise FileNotFoundError(f"The file {pddl_problem} does not exist.")
            else:
                if goal_num == 1 or goal_num == 2 or goal_num == 3:
                    pddl_domain = Path("domain.pddl")
                    # Run pyperplan with bfs
                    subprocess.run([
                        "pyperplan", str(pddl_domain), str(pddl_problem)],
                        check=True
                    )
                else:
                    pddl_domain = Path("custom_domain.pddl")
                    # Run pyperplan with greedy best first search rather than bfs
                    subprocess.run([
                        "pyperplan", "-s", "gbf", str(pddl_domain), str(pddl_problem)],
                        check=True
                    )
                # Save actions to .soln file & rename it 
                Path("problem.pddl.soln").rename("actions.soln")


