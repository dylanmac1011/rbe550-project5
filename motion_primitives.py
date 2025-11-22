import numpy as np
import random
import genesis as gs
#import planning as planner
import torch
from typing import Any
from scipy.spatial.transform import Rotation as R

class MotionPrimitives:
    def __init__(self, robot_: Any, scene_: Any, blocks_: Any):
        # ensure we have a RobotAdapter so the rest of the code can rely on a
        # stable interface (but attribute access is forwarded to the raw robot)
        self.robot = robot_
        #planner._ensure_adapter(robot_, scene_)
        self.scene = scene_
        #self.planner = planner.PlannerInterface(self.robot, self.scene)
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
        print(block)
        #Calculate pre-grasp pose just above block
        pregrasp_qpos, pre_grasp_pos, pre_grasp_quat = self.calcPreGraspPose(block)

        #Path plan for pre-grasp
        path = self.robot.plan_path(
        qpos_goal=pregrasp_qpos,
        num_waypoints=200)  # 2s duration

        #Follow path to pre-grasp state
        for waypoint in path:
            self.moveStep(waypoint)
        for i in range(100): #allow some time for robot to move to final position
            self.scene.step()
        
        #IK for grasp pose
        grasp_pos = pre_grasp_pos
        grasp_pos[2] -= 0.05
        grasp_qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=grasp_pos,
        quat=pre_grasp_quat)
        # gripper open
        grasp_qpos[-2:] = 0.04

        #move directly to grasp pose, no path planning needed
        self.moveTo(grasp_qpos)
        # close gripper
        self.grasp(grasp_qpos)
        #self.planner.attached_object = block
        self.moveTo(pregrasp_qpos, gripper=False)

    def put_down(self, block_str):
        x_pos = random.uniform(0.0,1.0)
        y_pos = random.uniform(0.0,1.0)
        z_pos = 0.11
        #Check if state is valid once OMPL works
        pos = np.array([x_pos,y_pos,z_pos])
        quat = np.array([0,0,0,0])
        qpos = self.robot.inverse_kinematics(
        link=self.robot.get_link("hand"),
        pos=pos)
        path = self.robot.plan_path(
        qpos_goal=qpos,
        num_waypoints=400)  # 4s duration

        #Follow path to random state
        for waypoint in path:
            self.moveStep(waypoint, gripper=False)
        for i in range(100): #allow some time for robot to move to final position
            self.scene.step()
        self.ungrasp(qpos)

    #Stacks blockA on blockB, assumes blockA in hand
    def stack(self, blockA_str,blockB_str):
        self.pick_up(blockA_str)
        blockA = self.blocks[blockA_str]
        blockB = self.blocks[blockB_str]
        prestack_qpos, pre_stack_pos, pre_stack_quat = self.calcPreGraspPose(blockB, stacking=True)

        #Path plan for pre-grasp
        path = self.robot.plan_path(
        qpos_goal=prestack_qpos,
        num_waypoints=400)  # 2s duration

        #Follow path to pre-grasp state
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
        self.put_down(block_str)