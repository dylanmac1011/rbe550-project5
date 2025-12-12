import numpy as np
import math
import random
import genesis as gs
import planning as planner
import torch
from typing import Any
from genesis.utils.misc import tensor_to_array
from scipy.spatial.transform import Rotation as R

class MotionPrimitives:
    def __init__(self, robot_: Any, scene_: Any, blocks_: Any, planner_: Any):
        # ensure we have a RobotAdapter so the rest of the code can rely on a
        # stable interface (but attribute access is forwarded to the raw robot)
        self.robot = robot_
        planner._ensure_adapter(robot_, scene_)
        self.planner = planner_
        self.scene = scene_
        self.blocks = blocks_
    
    motors_dof = np.arange(7)
    fingers_dof = np.arange(7, 9)
    
      
    def moveTo(self, qpos, gripper=True):
        if gripper:
            self.robot.control_dofs_position(qpos,np.arange(9))
        else:
            self.robot.control_dofs_position(qpos[:-2], self.motors_dof)
        for i in range(100):
            self.scene.step()
    
    def moveStep(self, qpos, gripper=True):
        if gripper:
            self.robot.control_dofs_position(qpos,np.arange(9))
        else:
            self.robot.control_dofs_position(qpos[:-2], self.motors_dof)
        self.scene.step()
    
    def getBlockPose(self, block):
        pos = block.get_pos()
        quat = block.get_quat()
        rot = R.from_quat(quat)
        r, p, y = rot.as_euler('xyz', degrees=False)
        return pos, r, p, y
    
    def calcPreGraspPose(self, block, stacking=False):

        block_pos, block_roll, block_pitch, block_yaw = self.getBlockPose(block)
        #Calculate pre-grasp pose just above block
        pre_grasp_pos = block_pos
        if stacking:
            z_adjust = 0.2
        else:
            z_adjust = 0.16
        print(f"block z: {pre_grasp_pos[2]}")
        pre_grasp_pos[2] = pre_grasp_pos[2] + z_adjust
        print(f"pre_grasp z: {pre_grasp_pos[2]}")
        pre_grasp_yaw = block_yaw + np.pi #Z axis rotated 180 degrees
        pre_grasp_R = R.from_euler('xyz', [block_roll, block_pitch, pre_grasp_yaw])
        pre_grasp_quat = pre_grasp_R.as_quat()
        pre_grasp_quat[1] = 1
        #IK for pre-grasp pose
        qpos = self.robot.inverse_kinematics(link=self.robot.get_link("hand"), 
            pos=pre_grasp_pos, quat=pre_grasp_quat, init_qpos=self.robot.get_qpos())
        qpos[-2:] = 0.04 # gripper open
        print(f"pre_grasp_pos: {pre_grasp_pos}")
        return qpos, pre_grasp_pos, pre_grasp_quat

    def grasp(self, qpos):
        self.robot.control_dofs_position(qpos[:-2], self.motors_dof)
        self.robot.control_dofs_force(np.array([-1, -1]), self.fingers_dof)
        print("grasping")
        for i in range(100):
            self.scene.step()

    def ungrasp(self, qpos):
        qpos[-2:] = 0.04
        self.robot.control_dofs_position(qpos, np.arange(9))
        self.planner.attached_object = None 
        for i in range(100):
            self.scene.step()

    def follow_path(self, qpos, gripper=True):
        path = self.planner.plan_path(
        qpos_goal=qpos,
        qpos_start=self.robot.get_qpos(),
        num_waypoints=200,
        planner="RRTstar")  # 2s duration

        #Follow path to pre-grasp state
        print("following path")
        for waypoint in path:
            self.moveStep(waypoint, gripper=gripper)
        for i in range(100): #allow some time for robot to move to final position
            self.scene.step()

    def generateBlockPos(self):
        x_pos = random.uniform(0.45,0.65)
        y_pos = random.uniform(-0.4, 0.4)
        z_pos = 0.18
        return x_pos, y_pos, z_pos
    
    def generateValidState(self):
        valid_state = False #Allow loop to start
        while not valid_state: #while a valid state has not been found
            x_pos, y_pos, z_pos = self.generateBlockPos() #generate random position on ground
            pos = np.array([x_pos,y_pos,z_pos])
            print(pos)
            valid_state = True #assume state is valid
            for block in self.blocks.values(): #for all blocks in the scene
                block_pos, r, p, y = self.getBlockPose(block)
                dist  = math.sqrt(((block_pos[0]-x_pos)**2) + ((abs(block_pos[1]-y_pos)**2)))
                if (dist < 0.15): #check that generated position is not near blocks
                    valid_state = False #if position is too close to other blocks, it is invalid
            print(valid_state)
            #if state is invalid, loop repeats
            #once all blocks are considered, if valid_state remains true, loop ends & function returns
        return x_pos, y_pos, z_pos

    def pick_up(self, block_str):
        #Retrieve block object from dictionary
        block = self.blocks[block_str]
        #print(block)
        #Calculate pre-grasp pose just above block
        pregrasp_qpos, pre_grasp_pos, pre_grasp_quat = self.calcPreGraspPose(block)
        pre_grasp_quat[1] = 1
        #print(f"quat: {pre_grasp_quat}")
        #print(f"pregrasp pos: {pre_grasp_pos}")
        grasp_pos = np.array([pre_grasp_pos[0], pre_grasp_pos[1], pre_grasp_pos[2] - 0.05])
        #print(f"grasp pos: {grasp_pos}")
        self.follow_path(pregrasp_qpos)
       # print(f"grasp pos: {grasp_pos}")
        grasp_qpos = self.robot.inverse_kinematics(init_qpos=self.robot.get_qpos(), 
            link=self.robot.get_link("hand"), pos=grasp_pos, quat=pre_grasp_quat)

        #move directly to grasp pose, no path planning needed
        self.planner.attached_object = block
        #print(f"attached_object:{self.planner.attached_object.idx}")
        self.moveTo(grasp_qpos, gripper=True)
        # close gripper
        self.grasp(grasp_qpos)

        grasp_pos[2] += 0.05
        post_grasp_qpos = self.robot.inverse_kinematics(init_qpos=self.robot.get_qpos(), 
            link=self.robot.get_link("hand"), pos=grasp_pos, quat=pre_grasp_quat)
        #self.planner.attached_object = block
        self.moveTo(post_grasp_qpos, gripper=False)

    def put_down(self, block_str):

        x_pos, y_pos, z_pos = self.generateValidState()
        #qpos_2, pos_2, quat = self.calcPreGraspPose(self.blocks[block_str])
        quat = np.array([0, 1, 0, 0])
        #Check if state is valid once OMPL works
        pos = np.array([x_pos,y_pos,z_pos])
        print(len(pos))
        print(pos)
        pre_place_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=pos,
        quat=quat)

        self.follow_path(pre_place_qpos, gripper=False)

        pos[2] -= 0.05
        place_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=pos,
        quat=quat)
        self.moveTo(place_qpos)
        self.ungrasp(place_qpos)
        pos[2] += 0.05
        post_place_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=pos,
        quat=quat)
        self.moveTo(post_place_qpos)

    #Stacks blockA on blockB, assumes blockA in hand
    def stack(self, blockA_str,blockB_str):
        #self.pick_up(blockA_str)
        blockA = self.blocks[blockA_str]
        blockB = self.blocks[blockB_str]
        prestack_qpos, pre_stack_pos, pre_stack_quat = self.calcPreGraspPose(blockB, stacking=True)
        pre_stack_quat[1] = 1
        stack_pos = np.array([pre_stack_pos[0], pre_stack_pos[1], pre_stack_pos[2] - 0.05])
        self.follow_path(prestack_qpos, gripper=False)

        stack_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=stack_pos,
        quat=pre_stack_quat)

        self.moveTo(stack_qpos, gripper=False)
        
        self.ungrasp(stack_qpos)
        stack_pos[2] += 0.05
        post_stack_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=stack_pos,
        quat=pre_stack_quat,)
        self.moveTo(post_stack_qpos)

    def primitiveFromString(self, string, line):
        index = line.find(string)
        match string:
            case "pick-up":
                block_index = index + 8
                block_str = line[block_index]
                print(f"picking up {block_str}")
                self.pick_up(block_str)
            case "put-down":
                block_index = index + 9
                block_str = line[block_index]
                print(f"putting down {block_str}")
                self.put_down(block_str)
            case "stack":
                block1_index = index + 6
                block1_str = line[block1_index]
                block2_index = index + 8
                block2_str = line[block2_index]
                print(f"stacking {block1_str}, {block2_str}")
                self.stack(block1_str, block2_str)
            case "unstack":
                block_index = index + 8
                block_str = line[block_index]
                print(f"unstacking {block_str}")
                self.pick_up(block_str)


    def runSolution(self, f_soln):
        primitives = ["pick-up", "put-down", "unstack", "stack"]
        try:
            with open(f_soln, 'r') as f:
                current_line = f.readline()
                print (current_line)
                while current_line:
                    for string in primitives:
                            if string in current_line:
                                self.primitiveFromString(string, current_line)
                                break #only 1 primitive per line
                    
                    current_line = f.readline()
                    #re-ground primitives
                    #re-plan if necessary
                    #call runSolution again with new .soln file
        except FileNotFoundError:
            print("Solution File Not Found")

    def runSolutionStep(self, f_soln):
        primitives = ["pick-up", "put-down", "unstack", "stack"]
        try:
            with open(f_soln, 'r') as f:
                current_line = f.readline()
                if current_line:
                    print (current_line)
                    for string in primitives:
                        if string in current_line:
                            self.primitiveFromString(string, current_line)
                            prev_step = current_line
                            break #only 1 primitive per line
                    return 0
                else:
                    return 1

        except FileNotFoundError:
            print("Solution File Not Found")
            