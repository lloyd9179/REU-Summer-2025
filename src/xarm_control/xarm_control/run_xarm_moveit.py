#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from xarm_msgs.srv import PlanPose, PlanExec
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from fpn_msgs.srv import BoolSrv
# from your_custom_pkg.srv import RobotState  # Ensure you import your custom service type

import numpy as np
import math
from scipy.spatial.transform import Rotation
import asyncio
import sys
import time
from rclpy.duration import Duration
import cv2
import os
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

class XarmMoveit(Node):
    def __init__(self):
        super().__init__('xarm_moveit')

        # Get the absolute path of the current file
        current_file_path = os.path.abspath(__file__)

        self.pose_plan_client = self.create_client(PlanPose, '/xarm_pose_plan')
        while not self.pose_plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for pose_plan_client')
        self.plan_exec_client = self.create_client(PlanExec, '/xarm_exec_plan')
        while not self.plan_exec_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for plan_exec_client')

        self.bridge = CvBridge()
        self.robot_state_sub = self.create_subscription(String,'/hunter/robot_state',self.robotstate_CB,10)
        self.kiyo_pro_sub = self.create_subscription(Image, '/kiyo_cam/image_raw',self.kiyoCB,1)
        self.arm_state_pub = self.create_publisher(String,'/hunter/data_collection_state',10)
        self.img_save_pub = self.create_publisher(Bool, '/hunter/img_save_flag',1)
        
        # Service server for arm state
        self.arm_state_srv = self.create_service(BoolSrv, '/kiyo_data_collection_service', self.arm_state_callback)

        self.data_start = 0
        self.capture_image = 0
        # time.sleep(15.0)

    def robotstate_CB(self, msg):
        self.robot_state = msg.data

        if self.data_start == 0:
            self.arm_state = 'stagnant'
        if self.data_start == 1:
            self.arm_state = 'capturing'
        arm_msg = String()
        arm_msg.data = self.arm_state
        self.arm_state_pub.publish(arm_msg)


    def kiyoCB(self, msg):
        self.image_data = msg
        if self.data_start == 0:
            self.arm_state = 'stagnant'
        if self.data_start == 1:
            self.arm_state = 'capturing'
        arm_msg = String()
        arm_msg.data = self.arm_state
        self.arm_state_pub.publish(arm_msg)

    # Service callback function
    def arm_state_callback(self, request, response):
        self.get_logger().info('Received robot state request: ' + str(request.boolean))
        
        if request.boolean and self.data_start == 0:
            self.data_start = 1
            # Ensure there is a running event loop before creating a task
            # loop = asyncio.get_event_loop()
            # loop = asyncio.new_event_loop()
            # asyncio.set_event_loop(loop)

            try:
                loop = asyncio.get_event_loop()
            except RuntimeError as e:
                if str(e).startswith('There is no current event loop in thread'):
                    loop = asyncio.new_event_loop()
                    asyncio.set_event_loop(loop)
                else:
                    raise
            
            if loop.is_running():
                loop.create_task(self.initialPose())
            else:
                # If there is no loop, start one
                asyncio.run(self.initialPose())

            response.success = True
            response.message = "Completed all poses of Kiyo pro"
        else:
            response.success = False
            response.message = "State not ready for initial pose execution."

        return response

    async def initialPose(self):
        self.initial_pose_xyz = [0.1979,0.0008,0.1996]
        init_pose_euler = [np.deg2rad(-179.9), np.deg2rad(0.0),np.deg2rad(0.1)] 
        self.init_rot_quaternion = Rotation.from_euler('xyz', init_pose_euler).as_quat()
        init_pose = Pose()
        init_pose.position = Point(x=self.initial_pose_xyz[0], y=self.initial_pose_xyz[1], z=self.initial_pose_xyz[2])
        init_pose.orientation = Quaternion(x=self.init_rot_quaternion[0],
                                           y=self.init_rot_quaternion[1],
                                           z=self.init_rot_quaternion[2],
                                           w=self.init_rot_quaternion[3])
        # self.get_logger().info('init_pose is: ' + str(init_pose))
        await asyncio.sleep(0.5)
        req = PlanPose.Request()
        req.target = init_pose
        future = self.pose_plan_client.call_async(req)
        
        self.get_logger().info('finished future.done while loop')
        if future.done:
            self.get_logger().info('finished creating pose plan with result ' + str(future.result()))
        
        req = PlanExec.Request()
        req.wait = True
        fin = self.plan_exec_client.call_async(req)

        if fin.done:
            self.get_logger().info("Move plan executed..")
        
        wait_time = 5
        await self.wait_arm(wait_time)

        self.get_logger().info("Started looping poses for data collection.....")
        await self.loop_poses()

        self.data_start = 0
        self.get_logger().info("Completed looping poses.....")

    async def loop_poses(self):
        # self.first_pose_xyz = [0.2003,-0.2169,0.0695]
        # self.second_pose_xyz = [0.2003,-0.2169,0.0695+0.1]
        # second_pose_euler = [np.deg2rad(-157.8), np.deg2rad(36.6),np.deg2rad(120.9)]
        # first_pose_euler = [np.deg2rad(-157.8), np.deg2rad(36.6),np.deg2rad(120.9+30)]
        # third_pose_euler = [np.deg2rad(-157.8), np.deg2rad(36.6),np.deg2rad(120.9-30)]
        

        # self.first_pose_xyz = [0.1979,0.0008,0.1996]
        # self.second_pose_xyz = [0.1979,0.0008,0.1996+0.1]
        # first_pose_euler = [np.deg2rad(-179.9), np.deg2rad(0.0),np.deg2rad(0.1+30)] 
        # second_pose_euler = [np.deg2rad(-179.9), np.deg2rad(0.0),np.deg2rad(0.1)] 
        # third_pose_euler = [np.deg2rad(-179.9), np.deg2rad(0.0),np.deg2rad(0.1-30)]        

        self.first_pose_xyz = [0.1608,-0.1727,0.1876]
        self.second_pose_xyz = [0.1608,-0.1727,0.1876+0.1]
        second_pose_euler = [np.deg2rad(176.1), np.deg2rad(6.2),np.deg2rad(177.7)]
        first_pose_euler = [np.deg2rad(176.1), np.deg2rad(6.2),np.deg2rad(177.7+30)]
        third_pose_euler = [np.deg2rad(176.1), np.deg2rad(6.2),np.deg2rad(177.7-30)]
        

        self.target_poses = [self.first_pose_xyz, self.second_pose_xyz]
        self.target_orientations = [first_pose_euler, second_pose_euler, third_pose_euler]

        self.image_counter = 1
        
        for target_xyz in self.target_poses:
            for target_orientation in self.target_orientations:
                self.target_rot_quaternion = Rotation.from_euler('xyz', target_orientation).as_quat()
                target_pose = Pose()
                target_pose.position = Point(x=target_xyz[0], y=target_xyz[1], z=target_xyz[2])
                target_pose.orientation = Quaternion(x=self.target_rot_quaternion[0],
                                                     y=self.target_rot_quaternion[1],
                                                     z=self.target_rot_quaternion[2],
                                                     w=self.target_rot_quaternion[3])
                # self.get_logger().info('target_pose is: ' + str(target_pose))
                await asyncio.sleep(0.5)
                req = PlanPose.Request()
                req.target = target_pose
                future = self.pose_plan_client.call_async(req)
                self.get_logger().info('finished future.done while loop')
                if future.done:
                    self.get_logger().info('finished creating pose plan with result ' + str(future.result()))
                
                req = PlanExec.Request()
                req.wait = True
                fin = self.plan_exec_client.call_async(req)

                if fin.done:
                    self.get_logger().info('Finished pose.....')
                
                wait_time = 9
                await self.wait_arm(wait_time)
                msg = Bool()
                msg.data = True
                self.img_save_pub.publish(msg)

                self.arm_state = 'capturing'
                arm_msg = String()
                arm_msg.data = self.arm_state
                self.arm_state_pub.publish(arm_msg)

                wait_time = 3
                await self.wait_arm(wait_time)

                self.get_logger().info("Moving to next pose.....")

        self.arm_state = 'stagnant'
        arm_msg = String()
        arm_msg.data = self.arm_state
        self.arm_state_pub.publish(arm_msg)

    async def wait_arm(self, wait_time):
        self.get_logger().info("Waiting for "+ str(wait_time) + " seconds to capture image...")
        await asyncio.sleep(wait_time)
        

def main(args=None):
    rclpy.init(args=args)
    sys.stdout.write('calling main of run_xarm_moveit.py')
    xarm_moveit = XarmMoveit()
    xarm_moveit.get_logger().info('finished instantiation of XarmMoveit')
    rclpy.spin(xarm_moveit)
    xarm_moveit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

