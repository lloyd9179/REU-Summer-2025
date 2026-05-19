#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from mobile_valens_interfaces.msg import ServoCamera
import time
import numpy as np
import cv2
import os
from pathlib import Path
from fpn_msgs.srv import BoolSrv
from cv_bridge import CvBridge

class PanTiltPub(Node):
	def __init__(self):
		super().__init__('pan_tilt_pub')
		self.get_logger().info('pan tilt pub 1')
		print("pan tilt pub init2")
		self.publisher = self.create_publisher(ServoCamera, 'pi_comm/pan_tilt', 10)		
		self.pan_tilt_flag = self.create_subscription(String,'/hunter/robot_state',self.robot_state_flagCB,10)
		self.pan_tilt_state_pub = self.create_publisher(String,'/hunter/pi_comm_state',10)
		self.img_save_flag_realsense = self.create_publisher(Bool,'/hunter/img_save_flag_realsense',10)
		self.bridge = CvBridge()
		timer_period = 0.2
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
		self.pan_arr = [0.7, 0.0, -.7]
		self.tilt_arr = [0.4, 0.0 , -.4]
		self.robot_state = 'moving'
		self.pi_state_service = self.create_service(BoolSrv,'/realsense_data_collection_service',self.pi_state_callback)
		# time.sleep(15.0)e
	
	def robot_state_flagCB(self,msg):
		self.robot_state = msg.data


	def pi_state_callback(self, request, response):
		self.get_logger().info('Received pi state request: ' + str(request.boolean))
		if request.boolean:
			for pan in self.pan_arr:
				for tilt in self.tilt_arr:
					msg = ServoCamera()
					msg.camera_pan_rad = pan
					msg.camera_tilt_rad = tilt
					self.publisher.publish(msg)
					
					pan_tilt_state = 'capturing'
					msg = String()
					msg.data = pan_tilt_state
					self.pan_tilt_state_pub.publish(msg)

					img_save_flag = Bool()
					img_save_flag.data = True
					self.img_save_flag_realsense.publish(img_save_flag)
					
					time.sleep(2.0)
			pan_tilt_state = 'stagnant'
			msg = String()
			msg.data = pan_tilt_state
			self.pan_tilt_state_pub.publish(msg)
			self.robot_state = 'moving'

			response.success = True
			response.message = "Completed all poses of realsense..."
		else:
			response.success = False
			response.message = "State not ready for initial pose execution."

		return response

	def timer_callback(self):
		if self.robot_state == 'moving':
			msg = String()
			msg.data = 'stagnant'
			self.pan_tilt_state_pub.publish(msg)
							
def main(args=None):
	print("pan tilt pub 1")
	rclpy.init(args = args)
	print("pan tilt pub 2")
	pantiltpub = PanTiltPub()
	print("pan tilt pub 3")
	rclpy.spin(pantiltpub)
	pantiltpub.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
		
