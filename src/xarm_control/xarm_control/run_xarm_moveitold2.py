 #!/usr/bin/env python

import rclpy
from rclpy.node import Node
from xarm_msgs.srv import PlanPose, PlanExec
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from fpn_msgs.srv import BoolSrv

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

        # Get the absolute path of the current file
        current_file_path = os.path.abspath(__file__)

        # Get the directory of the current file
        self.base_direc = os.path.dirname(current_file_path) + '/data_collection_images'

        
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
        self.data_start = 0
        self.capture_image = 0


    def robotstate_CB(self,msg):
        self.robot_state = msg.data

        if self.data_start == 0:
            self.arm_state = 'stagnant'
        if self.data_start ==1 :
            self.arm_state = 'capturing'
        arm_msg = String()
        arm_msg.data = self.arm_state
        self.arm_state_pub.publish(arm_msg)

        
        if self.robot_state == 'data_collection' and self.data_start==0:
            self.data_start = 1
            # Ensure there is a running event loop before creating a task
            loop = asyncio.get_event_loop()
            if loop.is_running():
                loop.create_task(self.initialPose())
            else:
                # If there is no loop, start one
                asyncio.run(self.initialPose())
    
    def kiyoCB(self,msg):
        # self.get_logger().info("MSG recieved:::")
        # Convert ROS Image to NumPy array
        self.image_data = msg
        if self.data_start == 0:
            self.arm_state = 'stagnant'
        if self.data_start ==1 :
            self.arm_state = 'capturing'
        arm_msg = String()
        arm_msg.data = self.arm_state
        self.arm_state_pub.publish(arm_msg)

        # if self.capture_image == 1:
        #     self.process_image(msg,self.save_folder, self.image_counter)
    

    # def process_image(self,msg,save_folder,image_counter):
    #     self.capture_image = 0
    #     # np_arr = np.frombuffer(msg.data, np.uint8)
        
    #     # # Reshape the array into the appropriate dimensions
    #     # image = np_arr.reshape(msg.height, msg.width, -1)

    #     # # Convert from RGB to BGR if necessary (OpenCV format)
    #     # if msg.encoding == 'rgb8':
    #     #     image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    #     # msg = self.image_data
    #     image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  # 'bgr8' is common for color images
            

    #     # Ensure the save folder exists
    #     if not os.path.exists(save_folder):
    #         os.makedirs(save_folder)
        
    #     # cv2.imshow("img",image)
    #     # cv2.waitKey(2000)
    #     img_save_path = save_folder+f'/img_{image_counter}.jpg'
    #     self.get_logger().info(str(img_save_path))
    #     cv2.imwrite(img_save_path,image)


    async def initialPose(self):
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
        
        wait_time = 2
        await self.wait_arm(wait_time)

        self.get_logger().info("Started looping poses for data collection.....")
        await self.loop_poses()

        self.data_start = 0


    
    async def loop_poses(self):

        self.first_pose_xyz = [0.2003,-0.2169,0.0695]
        self.second_pose_xyz = [0.2003,-0.2169,0.0695+0.1]
        second_pose_euler = [np.deg2rad(-157.8), np.deg2rad(36.6),np.deg2rad(120.9)]
        first_pose_euler = [np.deg2rad(-157.8), np.deg2rad(36.6),np.deg2rad(120.9+30)]
        third_pose_euler = [np.deg2rad(-157.8), np.deg2rad(36.6),np.deg2rad(120.9-30)]
        self.target_poses = [self.first_pose_xyz, self.second_pose_xyz]
        self.target_orientations = [first_pose_euler,second_pose_euler,third_pose_euler]

        items = os.listdir(self.base_direc)
        
        # Count how many of those items are directories
        folder_count = sum(1 for item in items if os.path.isdir(os.path.join(self.base_direc, item)))
        self.image_counter = 1
        
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
                # while not future.done:
                #     self.get_logger().info('waiting for plan to finish')
                self.get_logger().info('finished future.done while loop')
                if future.done:
                    self.get_logger().info('finished creating pose plan with result ' + str(future.result()))
                req = PlanExec.Request()
                req.wait = True
                fin = self.plan_exec_client.call_async(req)

                if fin.done:
                    self.get_logger().info('Finished pose.....')
                
                # self.save_folder = self.base_direc + f'/data_collection_{folder_count+1}'
                # self.process_image(self.image_data,self.save_folder, self.image_counter)
                # self.image_counter+= 1 
                
                # self.capture_image = 1



                wait_time = 10
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


        self.arm_state = 'stagnant'
        arm_msg = String()
        arm_msg.data = self.arm_state
        self.arm_state_pub.publish(arm_msg)

    async def wait_arm(self,wait_time):
        waiting_time = wait_time
        self.get_logger().info("Waiting for "+ str(waiting_time) + " seconds to capture image...")
        await asyncio.sleep(10)
        self.get_logger().info("Moving to next pose.....")

    # async def wait_arm(self, wait_time):
    #     self.get_logger().info(f"Waiting for {wait_time} seconds to capture image...")
        
    #     # Non-blocking sleep: does not block the event loop
    #     for i in range(wait_time):
    #         self.get_logger().info(f"Waiting... {i+1} seconds.")
    #         await asyncio.sleep(1)  # Sleep for 1 second at a time, allowing other callbacks to run

    #     self.get_logger().info("Moving to next pose.....")

                    
def main(args=None):
    rclpy.init(args=args)
    sys.stdout.write('calling main of run_xarm_moveit.py')
    xarm_moveit = XarmMoveit()
    xarm_moveit.get_logger().info('finished instantiation of XarmMoveit')
    # asyncio.run(xarm_moveit.initialPose())
    rclpy.spin(xarm_moveit)
    xarm_moveit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
