#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from xarm_msgs.srv import SetInt16ById, SetInt16, MoveHome, MoveCartesian

import numpy as np

import sys
import time

class ControlXarm(Node):
    
    def __init__(self):
        super().__init__('xarm_control')

        self.motion_en_client = self.create_client(SetInt16ById, '/ufactory/motion_enable')
        while not self.motion_en_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('motion_enable not available')
        self.mode_client = self.create_client(SetInt16, '/ufactory/set_mode')
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_mode not available')
        self.state_client = self.create_client(SetInt16, '/ufactory/set_state')
        while not self.state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_state not available')
        
        req = SetInt16ById.Request()
        req.id = 8
        req.data = 1
        self.motion_en_client.call_async(req)
        req = SetInt16.Request()
        req.data = 0
        self.mode_client.call_async(req)
        self.state_client.call_async(req)
        
        # self.arm_start_pose = [0.0653,-0.2175,0.2092,-177,0.1,176.8]
        self.start_pose_step = [0.261, -0.277, -0.346, np.deg2rad(-133.9), np.deg2rad(-43.1), np.deg2rad(-107)]
        self.arm_start_pose = [0.0,-0.20,0.20,-3.14,0.0,1.57]


        self.move_home_client = self.create_client(MoveHome, '/ufactory/move_gohome')
        while not self.move_home_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('move_gohome not available')
        
        req = MoveHome.Request()
        req.speed = 0.7
        req.acc = 3.5
        req.mvtime = 0.0
        future = self.move_home_client.call_async(req)
        # rclpy.spin_until_future_complete(self, future)

        self.set_pose_client = self.create_client(MoveCartesian, '/ufactory/set_position')
        while not self.set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_position not available')

        self.initial_pose()
        
    def initial_pose(self):
        self.get_logger().info('setting init pose')
        req = MoveCartesian.Request()
        req.pose = self.start_pose_step
        req.speed = 0.7
        req.acc = 3.5
        req.mvtime = 0.0
        self.set_pose_client.call_async(req)
        self.get_logger().info('call for start pose step')
        req.pose = self.arm_start_pose
        # self.set_pose_client.call_async(req)
        self.get_logger().info('call for start pose end')
    
    def set_pose(self, pose_goal):
        self.get_logger().info('within set_pose')
        req = MoveCartesian.Request()
        req.pose = pose_goal
        req.speed = 50.0
        req.acc = 500.0
        req.mvtime = 0.0
        return self.set_pose_client.call_async(req)
        # self.get_logger().info('finishing set_pose')

    def cycle_poses(self):
        # TODO
        self.get_logger().info('tbd')


def main(args=None):
    rclpy.init(args=args)
    xarm_controller = ControlXarm()
    # goal_pose = [250.0, 0.0, 250.0, 3.14, 0.0, 0.0]
    # resp = xarm_controller.set_pose(goal_pose)
    # xarm_controller.initial_pose()
    rclpy.spin(xarm_controller)
    xarm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

