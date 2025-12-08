import numpy as np
import math
import random
import genesis as gs
import planning as planner
import torch
from typing import Any
from scipy.spatial.transform import Rotation as R

class MotionPrimitives:
    def __init__(self, robot_: Any, scene_: Any, blocks_: Any):
        # ensure we have a RobotAdapter so the rest of the code can rely on a
        # stable interface (but attribute access is forwarded to the raw robot)
        self.robot = robot_
        planner._ensure_adapter(robot_, scene_)
        self.scene = scene_
        self.planner = planner.PlannerInterface(self.robot, self.scene)
        self.blocks = blocks_
        self.attached_object = None
    
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
        pre_grasp_pos[2] += z_adjust
        pre_grasp_yaw = block_yaw + np.pi #Z axis rotated 180 degrees
        pre_grasp_R = R.from_euler('xyz', [block_roll, block_pitch, pre_grasp_yaw])
        pre_grasp_quat = pre_grasp_R.as_quat()
        #IK for pre-grasp pose
        qpos = self.robot.inverse_kinematics(link=self.robot.get_link("hand"), pos=pre_grasp_pos, quat=pre_grasp_quat)
        qpos[-2:] = 0.04 # gripper open
        return qpos, pre_grasp_pos, pre_grasp_quat

    def grasp(self, qpos):
        self.robot.control_dofs_position(qpos[:-2], self.motors_dof)
        self.robot.control_dofs_force(np.array([-1, -1]), self.fingers_dof)
        for i in range(100):
            self.scene.step()

    def ungrasp(self, qpos):
        qpos[-2:] = 0.04
        self.robot.control_dofs_position(qpos, np.arange(9))
        for i in range(100):
            self.scene.step()

        
    def pick_up(self, block_str):
        #Retrieve block object from dictionary
        block = self.blocks[block_str]
        #print(block)
        #Calculate pre-grasp pose just above block
        pregrasp_qpos, pre_grasp_pos, pre_grasp_quat = self.calcPreGraspPose(block)
        #print(f"quat: {pre_grasp_quat}")
        #print(f"pos: {pre_grasp_pos}")

        #Path plan for pre-grasp
        path = self.robot.plan_path(
        qpos_goal=pregrasp_qpos,
        num_waypoints=300)  # 2s duration

        print("following path")
        #Follow path to pre-grasp state
        for waypoint in path:
            self.moveStep(waypoint)
        for i in range(100): #allow some time for robot to move to final position
            self.scene.step()
        
        #IK for grasp pose
        #grasp_pos = pre_grasp_pos
        pre_grasp_pos[2] -= 0.05
        print(f"pos: {pre_grasp_pos}")
        grasp_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=pre_grasp_pos,
        quat=pre_grasp_quat)

        #move directly to grasp pose, no path planning needed
        self.moveTo(grasp_qpos)
        # close gripper
        self.grasp(grasp_qpos)
        #self.planner.attached_object = block
        self.moveTo(pregrasp_qpos, gripper=False)

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
                if (dist < 0.1): #check that generated position is not near blocks
                    valid_state = False #if position is too close to other blocks, it is invalid
            print(valid_state)
            #if state is invalid, loop repeats
            #once all blocks are considered, if valid_state remains true, loop ends & function returns
        return x_pos, y_pos, z_pos

    def put_down(self, block_str):

        x_pos, y_pos, z_pos = self.generateValidState()
        qpos_2, pos_2, quat = self.calcPreGraspPose(self.blocks[block_str])
        #Check if state is valid once OMPL works
        pos = np.array([x_pos,y_pos,z_pos])
        print(len(pos))
        print(pos)
        pre_place_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=pos,
        quat=quat)
        path = self.robot.plan_path(
        qpos_goal=pre_place_qpos,
        num_waypoints=400)  # 4s duration

        #Follow path to random state
        print("following path")
        for waypoint in path:
            self.moveStep(waypoint, gripper=False)
        for i in range(100): #allow some time for robot to move to final position
            self.scene.step()

        pos[2] -= 0.05
        place_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=pos,
        quat=quat)

        self.moveTo(place_qpos)
        self.ungrasp(place_qpos)
        self.moveTo(pre_place_qpos)

    #Stacks blockA on blockB, assumes blockA in hand
    def stack(self, blockA_str,blockB_str):
        #self.pick_up(blockA_str)
        blockA = self.blocks[blockA_str]
        blockB = self.blocks[blockB_str]
        prestack_qpos, pre_stack_pos, pre_stack_quat = self.calcPreGraspPose(blockB, stacking=True)

        #Path plan for pre-grasp
        path = self.robot.plan_path(
        qpos_goal=prestack_qpos,
        num_waypoints=300)  # 2s duration

        #Follow path to pre-grasp state
        print("following path")
        for waypoint in path:
            self.moveStep(waypoint, gripper=False)
        for i in range(100): #allow some time for robot to move to final position
            self.scene.step()

        stack_pos = pre_stack_pos
        stack_pos[2] -= 0.05
        stack_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=stack_pos,
        quat=pre_stack_quat)
        self.moveTo(stack_qpos, gripper=False)
        
        self.ungrasp(stack_qpos)
        stack_pos[2] += 0.1
        stack_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=stack_pos,
        quat=pre_stack_quat,)
        self.moveTo(stack_qpos)
        
    def unstack(self, block_str):
        self.pick_up(block_str)
        #self.put_down(block_str)

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
            