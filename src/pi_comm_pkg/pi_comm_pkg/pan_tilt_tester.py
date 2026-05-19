#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from mobile_valens_interfaces.msg import ServoCamera
import time

class PanTiltPub(Node):
	def __init__(self):
		super().__init__('pan_tilt_test')
		self.publisher = self.create_publisher(ServoCamera, 'pi_comm/pan_tilt', 10)
		timer_period = 2
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0
		self.pan_arr = [0.0, .7, 0.0, -.7]
		self.tilt_arr = [.4, 0.0 , -.4, 0.0]
	
	def timer_callback(self):
		#self.get_logger().info("running timer_callback()")
		msg = ServoCamera()
		msg.camera_pan_rad = self.pan_arr[self.i]
		msg.camera_tilt_rad = self.tilt_arr[self.i]
		self.publisher.publish(msg)
		self.i = (self.i + 1) % 4

def main(args=None):
	rclpy.init(args = args)
	pantiltpub = PanTiltPub()
	rclpy.spin(pantiltpub)
	pantiltpub.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
		

