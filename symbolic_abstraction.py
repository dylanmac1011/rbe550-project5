import numpy as np
from robot_adapter import RobotAdapter
from scipy.spatial.transform import Rotation as R


def generate_pddl(scene, franka, BlocksState, SlotsState):
    """Generate a pddl file based on provided scene."""

    # Get all blocks in single string, separated by a space 
    blocks = " ".join(BlocksState.keys())

    # Get all slots in a single string, separated by a space
    slots = " ".join(SlotsState.keys())

    # Define all initial conditions of predicates (OG)
    hand_empty =  ""
    holding = ""
    clear = ""
    on_table = ""
    on = ""

    # Define new predicates for slots and blocks
    slot_empty = ""
    slot_occupied = ""
    block_unused = "" 
    block_used = ""
    grid_empty = ""

    # Get pose of franka's end effector
    end_effector = franka.get_link("hand")
    ee_pos = end_effector.get_pos()
    ee_quat = end_effector.get_quat()
    ee_R = R.from_quat(ee_quat)
    ee_roll, ee_pitch, ee_yaw = ee_R.as_euler('xyz', degrees=False)
    
    # HOLDING(A) - Check if the robot is holding a block based on pose
    valid_pos = False
    valid_quat = False
    valid_grip = False
    # Loop through blocks to determine if one is held
    for key, block in BlocksState.items():
        # Check if positions are valid for holding
        block_pos = block.get_pos()
        if abs(ee_pos[0] - block_pos[0]) < 0.01:
            if abs(ee_pos[1] - block_pos[1]) < 0.01:
                # Note: Z offset required between EE and block of roughly 0.11
                if abs(ee_pos[2] - block_pos[2]) - 0.11 < 0.01:
                    valid_pos = True
        # Check if orientations are valid for holding (only if positions are valid)
        if (valid_pos):
            # Get orientations
            block_quat = block.get_quat()
            block_R = R.from_quat(block_quat)
            block_roll, block_pitch, block_yaw = block_R.as_euler('xyz', degrees=False)
            # Check z rotated by 180 degrees (EE pointing down at block) & normalize
            delta_yaw = (ee_yaw - block_yaw + np.pi) % (2*np.pi) - np.pi
            if abs(delta_yaw - np.pi) < 0.1:
                valid_quat = True
        # Check if gripping if pose deemed valid using contact forces
        if (valid_pos and valid_quat):
            print("checking collision")
            # TODO: Consider moving depending on how expensive this collision check is
            # Positions of grippers
            if (abs((franka.get_qpos())[-1]) - 0.02 < 0.005 and abs((franka.get_qpos())[-2]) - 0.02 < 0.005):
                valid_grip = True
            # Add initial condition if valid grip
            if valid_grip:
                # TODO: note dangling space
                holding = "(holding " + key + ")"
                # Break loop since can only be holding one block at a time
                break
    

    # HANDEMPTY() - if not holding any blocks -> hand is empty
    if holding == "":
        hand_empty += "(handempty)"

    # ONTABLE(A) - Check if blocks are on table based on z coordinate
    # Create boolean list that indicates if a specific block is on the table
    on_table_bools = [False] * len(BlocksState)
    i = 0
    for key, block in BlocksState.items():
        block_pos = block.get_pos()
        if abs(block_pos[2] - 0.02) < 0.001:
            on_table += "(ontable " + key + ") "
            on_table_bools[i] = True
        i += 1

    # TODO: There is most likely better and more efficient logic for this check
    # ON(A,B) - check if all blocks not on table are on another block
    j = 0
    for top_key, top_block in BlocksState.items():
        # Check if block not on the table
        if on_table_bools[j] == False:
            top_pos = top_block.get_pos()
            k = 0
            for bottom_key, bottom_block in BlocksState.items():
                # Don't compare block against itself
                if (j != k):
                    bottom_pos = bottom_block.get_pos()
                    # Check if "same" x and y, proper z offset (0.04)
                    if abs(top_pos[0] - bottom_pos[0]) < 0.01:
                        if abs(top_pos[1] - bottom_pos[1]) < 0.01:
                            # Note: Z offset required between blocks is 0.04
                            if abs(top_pos[2] - bottom_pos[2]) - 0.04 < 0.005:
                                on += "(on " + top_key + " " + bottom_key + ") "
                                # Break since can only be on top of one block by definition
                                break
                k +=1
        j += 1
    
    # CLEAR(A) - check if a block has no blocks on top of it 
    # Check for all blocks that do not appear in the bottom position in "ON(A,B)"
    # Use logic that the bottom block always appears right before ")"
    bottom_blocks_indices = [index - 1 for index, char in enumerate(on) if char == ")"]
    bottom_blocks = [on[index] for index in bottom_blocks_indices]
    for key, block in BlocksState.items():
        if key not in bottom_blocks:
            clear += "(clear " + key + ") "

    # SLOTOCCUPIED(S) - checks if a slot is occupied (checks if a block has same location)
    # BLOCKUSED(B,S) - checks if a block is used (in a slot)
    # SLOTEMPTY(S) - checks if a slot is empty (no block at that location)
    # GRIDEMPTY() - checks if the entire grid is empty
    empty_count = 0
    for key_slot, slot in SlotsState.items():
        empty = True
        for key_block, block in BlocksState.items():
            if np.allclose(slot, block.get_pos(), atol=0.001):
                slot_occupied += "(filled " + key_slot + ") "
                block_used += "(in " + key_block + " " + key_slot + ") "
                empty = False
        if empty:
            slot_empty += "(empty " + key_slot + ") "
            empty_count = empty_count + 1
    
    if empty_count == 6 or empty_count == 10:
        grid_empty = "(gridempty) "

    # BLOCKUNUSED(B) - checks if a block is unused (not in a slot)
    for key_block, block in BlocksState.items():
        unused = True
        for key_slot, slot in SlotsState.items():
            if np.allclose(slot, block.get_pos(), atol=0.001):
                unused = False
        if unused:
            block_unused += "(unused " + key_block + ") "


    # Create the pddl file (can be treated as txt file)
    with open("problem.pddl", "w") as f:
        f.write("(define (problem BLOCKS PROBLEM)\n")
        f.write("(:domain BLOCKS)\n")
        f.write("(:objects " + blocks + " - block)\n")
        f.write("(:INIT " + on_table + on + clear + holding + slot_occupied + slot_empty + block_used + block_unused + grid_empty + hand_empty + ")\n")
        # TODO: Adjust goal
        f.write("(:goal (AND (on g b) (on r g) (on m c) (on y m)))\n)")
        # f.write("(:goal (AND (on r g) (on b r) (on y b) (on m y)))\n)")




