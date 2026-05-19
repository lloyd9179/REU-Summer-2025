 #!/usr/bin/env python

import rclpy
from rclpy.node import Node
from xarm_msgs.srv import PlanPose, PlanExec
from geometry_msgs.msg import Pose, Point, Quaternion

import numpy as np
import math
from scipy.spatial.transform import Rotation
import asyncio
import sys
import time
from rclpy.duration import Duration


class XarmMoveit(Node):
    def __init__(self):
        super().__init__('xarm_moveit')

        # self.motion_en_client = self.create_client(SetInt16ById, '/ufactory/motion_enable')
        # while not self.motion_en_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('motion_enable not available')
        # self.mode_client = self.create_client(SetInt16, '/ufactory/set_mode')
        # while not self.mode_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('set_mode not available')
        # self.state_client = self.create_client(SetInt16, '/ufactory/set_state')
        # while not self.state_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('set_state not available')
        
        # req = SetInt16ById.Request()
        # req.id = 8
        # req.data = 1
        # self.motion_en_client.call_async(req)
        # req = SetInt16.Request()
        # req.data = 0
        # self.mode_client.call_async(req)
        # self.state_client.call_async(req)
        

        self.pose_plan_client = self.create_client(PlanPose, '/xarm_pose_plan')
        while not self.pose_plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for pose_plan_client')
        self.plan_exec_client = self.create_client(PlanExec, '/xarm_exec_plan')
        while not self.plan_exec_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for plan_exec_client')

    def initialPose(self):
        # self.initial_pose_xyz = [0.0, -0.2, 0.2]
        # init_pose_euler = [-3.14, 0, 1.57]
        # self.start_pose_step = [0.261, -0.277, -0.346, np.deg2rad(-133.9), np.deg2rad(-43.1), np.deg2rad(-107)]
        # self.initial_pose_xyz = self.start_pose_step[:3]
        # init_pose_euler = self.start_pose_step[3:]
        # self.initial_pose_xyz = [0.2831,-0.1273,0.1648]
        # init_pose_euler = [np.deg2rad(163.9), np.deg2rad(28),np.deg2rad(55.2)]
        self.initial_pose_xyz = [0.2003,-0.2169,0.0695]
        init_pose_euler = [np.deg2rad(-157.8), np.deg2rad(36.6),np.deg2rad(120.9)]
        self.init_rot_quaternion = Rotation.from_euler('xyz', init_pose_euler).as_quat()
        init_pose = Pose()
        init_pose.position = Point(x=self.initial_pose_xyz[0], y=self.initial_pose_xyz[1], z=self.initial_pose_xyz[2])
        init_pose.orientation = Quaternion(x=self.init_rot_quaternion[0],
                                           y=self.init_rot_quaternion[1],
                                           z=self.init_rot_quaternion[2],
                                           w=self.init_rot_quaternion[3])
        self.get_logger().info('init_pose is: ' + str(init_pose))
        req = PlanPose.Request()
        # self.get_logger().info("Request", req)
        req.target = init_pose
        future = self.pose_plan_client.call_async(req)
        while not future.done:
            self.get_logger().info('waiting for plan to finish')
        self.get_logger().info('finished future.done while loop')
        if future.done:
            self.get_logger().info('finished creating pose plan with result ' + str(future.result()))
        req = PlanExec.Request()
        req.wait = True
        fin = self.plan_exec_client.call_async(req)

        while not fin.done:
            self.get_logger().info('waiting for finish')

        self.get_logger().info("Started looping posessssss.....")
        self.loop_poses()
    
    def loop_poses(self):

        self.first_pose_xyz = [0.2003,-0.2169,0.0695]
        self.second_pose_xyz = [0.2003,-0.2169,0.0695+0.1]
        second_pose_euler = [np.deg2rad(-157.8), np.deg2rad(36.6),np.deg2rad(120.9)]
        first_pose_euler = [np.deg2rad(-157.8), np.deg2rad(36.6),np.deg2rad(120.9+30)]
        third_pose_euler = [np.deg2rad(-157.8), np.deg2rad(36.6),np.deg2rad(120.9-30)]
        self.target_poses = [self.first_pose_xyz, self.second_pose_xyz]
        self.target_orientations = [first_pose_euler,second_pose_euler,third_pose_euler]
    
        for target_xyz in self.target_poses:
            for target_orientation in self.target_orientations:
                self.target_rot_quaternion = Rotation.from_euler('xyz', target_orientation).as_quat()
                target_pose = Pose()
                target_pose.position = Point(x= target_xyz[0],y=target_xyz[1],z= target_xyz[2])
                target_pose.orientation = Quaternion(x=self.target_rot_quaternion[0],
                                                y=self.target_rot_quaternion[1],
                                                z=self.target_rot_quaternion[2],
                                                w=self.target_rot_quaternion[3])
                self.get_logger().info('target_pose is: ' + str(target_pose))
                req = PlanPose.Request()
                # self.get_logger().info("Request", req)
                req.target = target_pose
                future = self.pose_plan_client.call_async(req)
                while not future.done:
                    self.get_logger().info('waiting for plan to finish')
                self.get_logger().info('finished future.done while loop')
                if future.done:
                    self.get_logger().info('finished creating pose plan with result ' + str(future.result()))
                req = PlanExec.Request()
                req.wait = True
                fin = self.plan_exec_client.call_async(req)

                while not fin.done:
                    self.get_logger().info('waiting for finish')

                    
def main(args=None):
    rclpy.init(args=args)
    sys.stdout.write('calling main of run_xarm_moveit.py')
    xarm_moveit = XarmMoveit()
    xarm_moveit.get_logger().info('finished instantiation of XarmMoveit')
    xarm_moveit.initialPose()
    rclpy.spin(xarm_moveit)
    xarm_moveit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


#     # async def initialPose(self):
#     #     self.initial_pose_xyz = [0.2003,-0.2169,0.0695]
#     #     init_pose_euler = [np.deg2rad(-157.8), np.deg2rad(36.6),np.deg2rad(120.9)]
#     #     self.init_rot_quaternion = Rotation.from_euler('xyz', init_pose_euler).as_quat()
#     #     init_pose = Pose()
#     #     init_pose.position = Point(x=self.initial_pose_xyz[0], y=self.initial_pose_xyz[1], z=self.initial_pose_xyz[2])
#     #     init_pose.orientation = Quaternion(x=self.init_rot_quaternion[0],
#     #                                     y=self.init_rot_quaternion[1],
#     #                                     z=self.init_rot_quaternion[2],
#     #                                     w=self.init_rot_quaternion[3])
#     #     self.get_logger().info('init_pose is: ' + str(init_pose))
#     #     req = PlanPose.Request()
#     #     req.target = init_pose
#     #     future = self.pose_plan_client.call_async(req)
        
#     #     # Wait for the plan to finish
#     #     while not future.done():
#     #         self.get_logger().info('waiting for plan to finish')
#     #         await asyncio.sleep(0.1)  # Non-blocking sleep
        
#     #     self.get_logger().info('finished future.done while loop')
        
#     #     if future.done():
#     #         self.get_logger().info('finished creating pose plan with result ' + str(future.result()))
        
#     #     req = PlanExec.Request()
#     #     req.wait = True
#     #     await self.plan_exec_client.call_async(req)

#     #     # Sleep for 2 seconds before calling loop_poses
#     #     self.get_logger().info("Sleeping for 2 seconds before starting loop_poses...")
#     #     await asyncio.sleep(2)

#     #     self.get_logger().info("Started looping posessssss.....")
#     #     await self.loop_poses()  # Call loop_poses asynchronously

#     # async def loop_poses(self):
#     #     self.first_pose_xyz = [0.2003,-0.2169,0.0695]
#     #     self.second_pose_xyz = [0.2003,-0.2169,0.0695+0.1]
#     #     second_pose_euler = [np.deg2rad(-157.8), np.deg2rad(36.6),np.deg2rad(120.9)]
#     #     first_pose_euler = [np.deg2rad(-157.8), np.deg2rad(36.6),np.deg2rad(120.9+30)]
#     #     third_pose_euler = [np.deg2rad(-157.8), np.deg2rad(36.6),np.deg2rad(120.9-30)]
#     #     self.target_poses = [self.first_pose_xyz, self.second_pose_xyz]
#     #     self.target_orientations = [first_pose_euler, second_pose_euler, third_pose_euler]

#     #     for target_xyz in self.target_poses:
#     #         for target_orientation in self.target_orientations:
#     #             self.target_rot_quaternion = Rotation.from_euler('xyz', target_orientation).as_quat()
#     #             target_pose = Pose()
#     #             target_pose.position = Point(x=target_xyz[0], y=target_xyz[1], z=target_xyz[2])
#     #             target_pose.orientation = Quaternion(x=self.target_rot_quaternion[0],
#     #                                                 y=self.target_rot_quaternion[1],
#     #                                                 z=self.target_rot_quaternion[2],
#     #                                                 w=self.target_rot_quaternion[3])
#     #             self.get_logger().info('target_pose is: ' + str(target_pose))
#     #             req = PlanPose.Request()
#     #             req.target = target_pose
#     #             future = self.pose_plan_client.call_async(req)

#     #             # Wait for the plan to finish
#     #             while not future.done():
#     #                 self.get_logger().info('waiting for plan to finish')
#     #                 await asyncio.sleep(0.1)  # Non-blocking sleep
                
#     #             self.get_logger().info('finished future.done while loop')

#     #             if future.done():
#     #                 self.get_logger().info('finished creating pose plan with result ' + str(future.result()))
                
#     #             req = PlanExec.Request()
#     #             req.wait = False
#     #             await self.plan_exec_client.call_async(req)  # Execute the plan

#     #             # Sleep for 5 seconds after each plan_exec call
#     #             self.get_logger().info("Sleeping for 5 seconds after plan execution...")
#     #             await asyncio.sleep(5)  # Non-blocking sleep

#     # To execute initialPose in an asynchronous context, use asyncio.run() like this:
#     # asyncio.run(initialPose(self)) or ensure that it's called within an async event loop


# class XarmMoveit(Node):
#     def __init__(self):
#         super().__init__('xarm_moveit')

#         self.pose_plan_client = self.create_client(PlanPose, '/xarm_pose_plan')
#         while not self.pose_plan_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('waiting for pose_plan_client')

#         self.plan_exec_client = self.create_client(PlanExec, '/xarm_exec_plan')
#         while not self.plan_exec_client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('waiting for plan_exec_client')

#     def initialPose(self):
#         self.initial_pose_xyz = [0.2003, -0.2169, 0.0695]
#         init_pose_euler = [np.deg2rad(-157.8), np.deg2rad(36.6), np.deg2rad(120.9)]
#         self.init_rot_quaternion = Rotation.from_euler('xyz', init_pose_euler).as_quat()
#         init_pose = Pose()
#         init_pose.position = Point(x=self.initial_pose_xyz[0], y=self.initial_pose_xyz[1], z=self.initial_pose_xyz[2])
#         init_pose.orientation = Quaternion(x=self.init_rot_quaternion[0],
#                                            y=self.init_rot_quaternion[1],
#                                            z=self.init_rot_quaternion[2],
#                                            w=self.init_rot_quaternion[3])
#         self.get_logger().info('init_pose is: ' + str(init_pose))
#         req = PlanPose.Request()
#         req.target = init_pose
#         # Make synchronous call to the service
#         future = self.pose_plan_client.call(req)

#         self.get_logger().info('finished creating pose plan with result ' + str(future))

#         req_exec = PlanExec.Request()
#         req_exec.wait = True
#         # Make synchronous call to the service
#         future_exec = self.plan_exec_client.call(req_exec)

#         self.get_logger().info("Finished executing plan.")

#         # Sleep for 5 seconds
#         self.get_logger().info("Sleeping for 5 seconds before starting loop...")
#         time.sleep(5)
#         self.get_logger().info("Finished sleeping.")
#         self.loop_poses()

#     def loop_poses(self):
#         self.first_pose_xyz = [0.2003, -0.2169, 0.0695]
#         self.second_pose_xyz = [0.2003, -0.2169, 0.0695 + 0.1]
#         second_pose_euler = [np.deg2rad(-157.8), np.deg2rad(36.6), np.deg2rad(120.9)]
#         first_pose_euler = [np.deg2rad(-157.8), np.deg2rad(36.6), np.deg2rad(120.9 + 30)]
#         third_pose_euler = [np.deg2rad(-157.8), np.deg2rad(36.6), np.deg2rad(120.9 - 30)]
#         self.target_poses = [self.first_pose_xyz, self.second_pose_xyz]
#         self.target_orientations = [first_pose_euler, second_pose_euler, third_pose_euler]

#         for target_xyz in self.target_poses:
#             for target_orientation in self.target_orientations:
#                 self.target_rot_quaternion = Rotation.from_euler('xyz', target_orientation).as_quat()
#                 target_pose = Pose()
#                 target_pose.position = Point(x=target_xyz[0], y=target_xyz[1], z=target_xyz[2])
#                 target_pose.orientation = Quaternion(x=self.target_rot_quaternion[0],
#                                                      y=self.target_rot_quaternion[1],
#                                                      z=self.target_rot_quaternion[2],
#                                                      w=self.target_rot_quaternion[3])
#                 self.get_logger().info('target_pose is: ' + str(target_pose))
#                 req = PlanPose.Request()
#                 req.target = target_pose
#                 # Make synchronous call to the service
#                 future = self.pose_plan_client.call(req)

#                 self.get_logger().info('finished creating pose plan with result ' + str(future))

#                 req_exec = PlanExec.Request()
#                 req_exec.wait = True
#                 # Make synchronous call to the service
#                 future_exec = self.plan_exec_client.call(req_exec)

#                 self.get_logger().info("Finished executing plan.")

#                 # Sleep for 5 seconds after each execution
#                 self.get_logger().info("Sleeping for 5 seconds before next iteration...")
#                 time.sleep(5)
#                 self.get_logger().info("Finished sleeping.")

# def main(args=None):
#     rclpy.init(args=args)
#     sys.stdout.write('calling main of run_xarm_moveit.py')
    
#     # Create an instance of XarmMoveit
#     xarm_moveit = XarmMoveit()
#     xarm_moveit.get_logger().info('finished instantiation of XarmMoveit')
    
#     # Start the initial pose and loop
#     xarm_moveit.initialPose()  # Run initialPose synchronously
    
#     rclpy.spin(xarm_moveit)
#     xarm_moveit.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
