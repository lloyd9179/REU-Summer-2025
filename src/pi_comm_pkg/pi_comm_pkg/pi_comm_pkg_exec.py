#!/usr/bin/env python
"""
pi_comm_pkg_exec.py
Function: This program handles all of the i2c and spi communication for the SoftAgBot.
As commands arrive, they are queued up to prevent more than one device trying to use the bus at the same time.
This is a bigger issue due to our use of the multiplexor that only allows one device to talk at a time.
Author: Benjamin Walt
Date: 12/25/2022
Purpose: ROS2 SoftAgBot system integration project
Version: 0.1
"""


import rclpy
from rclpy.node import Node

# ~ from std_msgs.msg import String



"""
General Imports 
"""
# ~ import rospy
import queue as Q # This creates a queue that is protected with locks
# ~ import Queue as queue # This creates a queue that is protected with locks
import sys
#import RPi.GPIO as GPIO
import numpy as np
import time

"""
Equipment classes
"""
from .submodules.PCA9685 import PCA9685 as PCA # PWM driver
from .submodules.TCA9548A import TCA9548A as TCA
"""
Messages
"""

from mobile_valens_interfaces.msg import ServoCamera
from std_msgs.msg import Bool

"""
Setting up pin data from pin_out.yaml
"""

# Servo Channels
_CAMERA_PAN_channel = 14#rospy.get_param("/CAMERA_PAN")
_CAMERA_TILT_channel = 15#rospy.get_param("/CAMERA_TILT")

"""
Setting up pin data from parameters.yaml
"""
_OSCILLATOR_FREQ = 27100000#rospy.get_param("/oscillator_freq") # For PCA9685

_PAN_CORRECTION = 0#np.deg2rad(rospy.get_param("/pan_correction"))
_TILT_CORRECTION = 0.25 # np.deg2rad(rospy.get_param("/tilt_correction"))

class PiCommunications(Node):

	def __init__(self):
		super().__init__('pi_comm_node')
		# ~ rospy.init_node('pi_comm_node', anonymous=True)
		# ~ rospy.on_shutdown(self._shutdown_hook)
		self._queue = Q.Queue() # This is a FIFO used to store actions in the order they are recieved
		# Initilize all the classes
		#self._tca = TCA.TCA9548A()
		# The servo controller
		self.get_logger().info("running init")
		self._pwm = PCA.PCA9685()
		self._pwm.set_oscillator_freq(_OSCILLATOR_FREQ)
		pan_pwm = self._camera_servo_rad2pwm(0 + _PAN_CORRECTION) # Correction fixes any disparity in the zero position
		tilt_pwm = self._camera_servo_rad2pwm(0 + _TILT_CORRECTION)
		self._pwm.set_pwm(_CAMERA_PAN_channel, 0, pan_pwm)
		self._pwm.set_pwm(_CAMERA_TILT_channel, 0, tilt_pwm)
		
		#######################
		# Subscribers
		#######################
		# ~ self.subscription = self.create_subscription(
            # ~ String,
            # ~ 'topic',
            # ~ self.listener_callback,
            # ~ 10)
        # ~ self.subscription  # prevent unused variable warning
		self.create_subscription(ServoCamera, 'pi_comm/pan_tilt', self._camera_callback, 10)
		
		
		#######################
		# Publishers
		#######################
		# ~ self.publisher_ = self.create_publisher(String, 'topic', 10)
		
		#######################
		# State Data
		#######################
		self.rate = self.create_rate(1)
		# self._start_queue()

	"""
	Callback that adds a call to the queue to change the camera pose
	"""
	def _camera_callback(self, msg):
		self.get_logger().info("camera_callback")
		pan_pwm = self._camera_servo_rad2pwm(msg.camera_pan_rad + _PAN_CORRECTION) # Correction fixes any disparity in the zero position
		tilt_pwm = self._camera_servo_rad2pwm(msg.camera_tilt_rad + _TILT_CORRECTION)
		#self.get_logger().info("msg pan: " + str(msg.camera_pan_rad))
		#self.get_logger().info("msg tilt: " + str(msg.camera_tilt_rad))		
		#self._tca.select_i2c_device(_SERVO_channel)
		self._pwm.set_pwm(_CAMERA_PAN_channel, 0, pan_pwm)
		self._pwm.set_pwm(_CAMERA_TILT_channel, 0, tilt_pwm)
		self.rate.sleep()   ##comment out?
        # Add to queue
		# self._queue.put(["CAMERA_SERVO", pan_pwm, tilt_pwm])

	"""
	Pan and Tilt Servos only
	Coverts the desired position of the servo in radians to a PWM signal
	Only works for the specific type of servo used
	"""
	def _camera_servo_rad2pwm(self, radians):
		# For HS488HB Servo
		#MIN_USEC = 553.0
		#MAX_USEC = 2425.0
		#RANGE_RAD = 3.3161
		# For HS-7950TH Servo
		#MIN_USEC = 750.0
		#MAX_USEC = 2250.0
		#RANGE_RAD = np.deg2rad(150.)
		# For DS 3218 Sevo
		MIN_USEC = 500.0
		MAX_USEC = 2500.0
		RANGE_RAD = 4.7124	
		
		usec_per_rad = (MAX_USEC-MIN_USEC)/RANGE_RAD
		usec = ((MAX_USEC-MIN_USEC)/2.0) + MIN_USEC + radians*usec_per_rad
		usec = max(MIN_USEC, min(MAX_USEC, usec))
		pwm = int(round((usec/20000.0)*4095)) # 50hz -> 20ms pulse width
		return pwm


	"""
	This loop operates based on the queus. As items are added to the queue the are executed in FILO order.  When empty, it only does some housekeeping tasks and is otherwise
	sleeping.
	"""
	def _start_queue(self):
		while(rclpy.ok()):
			self.get_logger().info("outer while")
			while(not self._queue.empty()):
				self.get_logger().info("inner while");
				action = self._queue.get() # Should be a list of at least the device name and also any command values if needed
				device = action[0] # First element should contain the device name
					
				if(device == "CAMERA_SERVO"): # Control camera P&T
					self.get_logger().info("queue popping CAM_SERVO")
					self._tca.select_i2c_device(_SERVO_channel)
					self._pwm.set_pwm(_CAMERA_PAN_channel, 0, action[1])
					self._pwm.set_pwm(_CAMERA_TILT_channel, 0, action[2])
						

			self.rate.sleep()



def main(args=None):
    rclpy.init(args=args)

    pi_communications = PiCommunications()

    rclpy.spin(pi_communications)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pi_communications.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
