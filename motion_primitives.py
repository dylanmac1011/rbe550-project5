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
    def __init__(self, robot_: Any, scene_: Any, blocks_: Any):
        # ensure we have a RobotAdapter so the rest of the code can rely on a
        # stable interface (but attribute access is forwarded to the raw robot)
        self.robot = robot_
        #planner._ensure_adapter(robot_, scene_)
        #self.planner = planner_
        self.scene = scene_
        self.blocks = blocks_
    
    motors_dof = np.arange(7)
    fingers_dof = np.arange(7, 9)
    
      
    def moveTo(self, qpos, gripper=True):
        if gripper:
            self.robot.control_dofs_position(qpos,np.arange(9))
        else:
            self.robot.control_dofs_position(qpos[:-2], self.motors_dof)
        for i in range(50):
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
            z_adjust = 0.21
        print(f"block z: {pre_grasp_pos[2]}")
        pre_grasp_pos[2] = pre_grasp_pos[2] + z_adjust
        pre_grasp_pos[0] = pre_grasp_pos[0] + 0.005
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

    def calcPrePlacePose(self, block, direction):

        block_pos, block_roll, block_pitch, block_yaw = self.getBlockPose(block)
        #Calculate pre-grasp pose just above block
        pre_place_pos = block_pos
        pre_place_pos[2] += 0.22
        match direction:
            case "north":
                pre_place_pos[1] += 0.047
            case "south":
                pre_place_pos[1] -= 0.047
            case "east":
                pre_place_pos[0] += 0.047
            case "west":
                pre_place_pos[0] -= 0.047
            case "northeast":
                pre_place_pos[0] += 0.047
                pre_place_pos[1] += 0.047
            case "northwest":
                pre_place_pos[0] -= 0.047
                pre_place_pos[1] += 0.047
            case "southeast":
                pre_place_pos[0] += 0.047
                pre_place_pos[1] -= 0.047
            case "southwest":
                pre_place_pos[0] -= 0.047
                pre_place_pos[1] -= 0.047

        pre_place_yaw = block_yaw + np.pi #Z axis rotated 180 degrees
        pre_place_R = R.from_euler('xyz', [block_roll, block_pitch, pre_place_yaw])
        pre_place_quat = pre_place_R.as_quat()
        pre_place_quat[1] = 1
        #IK for pre-grasp pose
        qpos = self.robot.inverse_kinematics(link=self.robot.get_link("hand"), 
            pos=pre_place_pos, quat=pre_place_quat, init_qpos=self.robot.get_qpos())
        qpos[-2:] = 0.04 # gripper open
        print(f"pre_grasp_pos: {pre_place_pos}")
        return qpos, pre_place_pos, pre_place_quat

    def grasp(self, qpos):
        self.robot.control_dofs_position(qpos[:-2], self.motors_dof)
        self.robot.control_dofs_force(np.array([-1, -1]), self.fingers_dof)
        print("grasping")
        for i in range(50):
            self.scene.step()

    def ungrasp(self, qpos):
        qpos[-2:] = 0.04
        self.robot.control_dofs_position(qpos, np.arange(9))
        #self.planner.attached_object = None 
        for i in range(50):
            self.scene.step()

    def follow_path(self, qpos, gripper=True):
        path = self.robot.plan_path(
        qpos_goal=qpos,
        num_waypoints=200,
        planner="RRT") # 2s duration

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
        grasp_pos = tensor_to_array(pre_grasp_pos).copy()
        #print(f"quat: {pre_grasp_quat}")
        print(f"pregrasp pos: {pre_grasp_pos}")
        #self.follow_path(pregrasp_qpos)
        path = self.robot.plan_path(
        qpos_goal=pregrasp_qpos,
        num_waypoints=200,
        resolution=0.2)  # 2s duration

        print("following path")
        #Follow path to pre-grasp state
        for waypoint in path:
            self.moveStep(waypoint)
        for i in range(25): #allow some time for robot to move to final position
            self.scene.step()

        
        grasp_pos[2] -= 0.1
        grasp_qpos = self.robot.inverse_kinematics(init_qpos=self.robot.get_qpos(), 
            link=self.robot.get_link("hand"), pos=grasp_pos, quat=pre_grasp_quat)
        print(f"grasp pos: {grasp_pos}")
        path2 = self.robot.plan_path(
        qpos_goal=grasp_qpos,
        num_waypoints=50,
        resolution=0.2)  # 2s duration

        print("following path")
        #Follow path to pre-grasp state
        for waypoint in path2:
            self.moveStep(waypoint)
        for i in range(25): #allow some time for robot to move to final position
            self.scene.step()

        #self.moveTo(grasp_qpos, gripper=True)
        # close gripper
        self.grasp(grasp_qpos)

        grasp_pos[2] += 0.1
        post_grasp_qpos = self.robot.inverse_kinematics(init_qpos=self.robot.get_qpos(), 
            link=self.robot.get_link("hand"), pos=grasp_pos, quat=pre_grasp_quat)
        #self.planner.attached_object = block
        self.moveTo(post_grasp_qpos, gripper=False)

    def put_down(self, block_str):

        #self.moveTo()
        x_pos, y_pos, z_pos = self.generateValidState()
        #qpos_2, pos_2, quat = self.calcPreGraspPose(self.blocks[block_str])
        quat = np.array([0, 1, 0, 0])
        #Check if state is valid once OMPL works
        pos = np.array([x_pos,y_pos,z_pos])
        pre_place_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=torch.tensor(pos),
        quat=torch.tensor(quat))

        path = self.robot.plan_path(
        qpos_goal=pre_place_qpos,
        num_waypoints=200,
        resolution=0.2)  # 2s duration

        print("following path")
        #Follow path to pre-grasp state
        for waypoint in path:
            self.moveStep(waypoint, gripper=False)
        for i in range(25): #allow some time for robot to move to final position
            self.scene.step()

        pos[2] -= 0.05
        place_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=pos,
        quat=quat)
        path2 = self.robot.plan_path(
        qpos_goal=place_qpos,
        num_waypoints=50,
        resolution=0.2)  # 2s duration

        print("following path")
        #Follow path to pre-grasp state
        for waypoint in path2:
            self.moveStep(waypoint)
        for i in range(25): #allow some time for robot to move to final position
            self.scene.step()
        self.ungrasp(place_qpos)
        pos[2] += 0.1
        post_place_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=pos,
        quat=quat)
        self.moveTo(post_place_qpos)

    def place_first(self, block_str):
        quat = np.array([0, 1, 0, 0])
        pos = self.robot.forward_kinematics(qpos=self.robot.get_qpos(), 
            link=self.robot.get_link("hand"))
        print(len(pos))
        print(pos)
        pre_place_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=pos,
        quat=quat)

        path = self.robot.plan_path(
        qpos_goal=pre_place_qpos,
        num_waypoints=200)  # 2s duration

        print("following path")
        #Follow path to pre-grasp state
        for waypoint in path:
            self.moveStep(waypoint, gripper=False)
        for i in range(25): #allow some time for robot to move to final position
            self.scene.step()

        pos[2] -= 0.05
        place_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=pos,
        quat=quat)
        self.moveTo(place_qpos)
        self.ungrasp(place_qpos)
        pos[2] += 0.1
        post_place_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=pos,
        quat=quat)
        self.moveTo(post_place_qpos)

    #Stacks blockA on blockB, assumes blockA in hand
    def stack(self, blockA_str,blockB_str, shape=False):
        #self.pick_up(blockA_str)
        blockA = self.blocks[blockA_str]
        blockB = self.blocks[blockB_str]
        prestack_qpos, pre_stack_pos, pre_stack_quat = self.calcPreGraspPose(blockB, stacking=True)
        pre_stack_quat = np.array([0, 1, 0, 0])
        stack_pos = tensor_to_array(pre_stack_pos).copy()
        if shape:
            adjust = 0.0
        else:
            adjust = 0.04
        stack_pos[2] -= adjust

        path = self.robot.plan_path(
        qpos_goal=prestack_qpos,
        num_waypoints=200,
        resolution=0.2)  # 2s duration

        print("following path")
        #Follow path to pre-grasp state
        for waypoint in path:
            self.moveStep(waypoint, gripper=False)
        for i in range(25): #allow some time for robot to move to final position
            self.scene.step()
   
        stack_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=stack_pos,
        quat=pre_stack_quat)

        self.moveTo(stack_qpos, gripper=False)
        
        self.ungrasp(stack_qpos)
        stack_pos[2] += 0.1
        post_stack_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=stack_pos,
        quat=pre_stack_quat,)
        self.moveTo(post_stack_qpos)

    def place_direction(self, blockA_str, blockB_str, direction):
        blockA = self.blocks[blockA_str]
        blockB = self.blocks[blockB_str]
        preplace_qpos, pre_place_pos, pre_place_quat = self.calcPrePlacePose(blockB, direction=direction)
        pre_place_quat = np.array([0, 1, 0, 0])
        place_pos = tensor_to_array(pre_place_pos).copy()
        place_pos[2] -= 0.05

        path = self.robot.plan_path(
        qpos_goal=preplace_qpos,
        num_waypoints=200,
        resolution=0.2)  # 2s duration

        print("following path")
        #Follow path to pre-grasp state
        for waypoint in path:
            self.moveStep(waypoint, gripper=False)
        for i in range(100): #allow some time for robot to move to final position
            self.scene.step()
   
        place_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=place_pos,
        quat=pre_place_quat)

        self.moveTo(place_qpos, gripper=False)
        
        self.ungrasp(place_qpos)
        place_pos[2] += 0.1
        post_place_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=place_pos,
        quat=pre_place_quat,)
        self.moveTo(post_place_qpos)
       

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
            case "place-above":
                block1_index = index + 14
                block1_str = line[block1_index]
                block2_index = index + 16
                block2_str = line[block2_index]
                self.stack(block1_str, block2_str, shape=True)
            case "place-first":
                block_index = index + 12
                block_str = line[block_index]
                self.ungrasp(self.robot.get_qpos())
                #self.place_first(block_str)
            case "place-north":
                block1_index = index + 12
                block1_str = line[block1_index]
                block2_index = index + 14
                block2_str = line[block2_index]
                self.place_direction(block1_str, block2_str, "north")
            case "place-west":
                block1_index = index + 11
                block1_str = line[block1_index]
                block2_index = index + 13
                block2_str = line[block2_index]
                self.place_direction(block1_str, block2_str, "west")
            case "place-south":
                block1_index = index + 12
                block1_str = line[block1_index]
                block2_index = index + 14
                block2_str = line[block2_index]
                self.place_direction(block1_str, block2_str, "south")
            case "place-east":
                block1_index = index + 11
                block1_str = line[block1_index]
                block2_index = index + 13
                block2_str = line[block2_index]
                self.place_direction(block1_str, block2_str, "east")
            case "place-northeast":
                block1_index = index + 16
                block1_str = line[block1_index]
                block2_index = index + 18
                block2_str = line[block2_index]
                self.place_direction(block1_str, block2_str, "northeast")
            case "place-northwest":
                block1_index = index + 16
                block1_str = line[block1_index]
                block2_index = index + 18
                block2_str = line[block2_index]
                self.place_direction(block1_str, block2_str, "northwest")
            case "place-southeast":
                block1_index = index + 16
                block1_str = line[block1_index]
                block2_index = index + 18
                block2_str = line[block2_index]
                self.place_direction(block1_str, block2_str, "southeast")
            case "place-southwest":
                block1_index = index + 16
                block1_str = line[block1_index]
                block2_index = index + 18
                block2_str = line[block2_index]
                self.place_direction(block1_str, block2_str, "southwest")



    def runSolution(self, f_soln):
        primitives = ["pick-up", "put-down", "unstack", "stack", "place-first", \
        "place-northeast", "place-northwest", "place-southeast", "place-southwest","place-west", "place-north", "place-east", "place-south", "place-above"]
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
        primitives = ["pick-up", "put-down", "unstack", "stack", "place-first", \
        "place-northeast", "place-northwest", "place-southeast", "place-southwest","place-west", "place-north", "place-east", "place-south", "place-above"]
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
            