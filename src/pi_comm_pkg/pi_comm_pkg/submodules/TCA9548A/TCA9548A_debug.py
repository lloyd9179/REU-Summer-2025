#!/usr/bin/env python
"""
TCA9548A.py
Function: A simple class to operate a TCA9548A I2C multiplexor on an RPi with Python
Author: Benjamin Walt
Date: 9/30/2020
Purpose: SoftAgBot system integration project
Version: 0.2
This is a rework of Adafruit's software:
https://github.com/adafruit/Adafruit_CircuitPython_TCA9548A
https://www.adafruit.com/product/2717
"""
import smbus

_DEVICE_ADDRESS = 0x70


class TCA9548A():
	"""Class to control the TCA9548A I2C multiplexor"""
	def __init__(self):
		self._address = _DEVICE_ADDRESS
		print("DEBUG: Connected to TCA9548A in debug mode")
			
		self._current_channel = 0

	def select_i2c_device(self, i2c_channel):
		"""Select the desired channel 0-7"""
		if i2c_channel < 0 or i2c_channel >7:
			print("DEBUG: TCA9548A channel out of range.  Cannot set to {}".format(i2c_channel))
			i2c_channel = self._current_channel
		else:
			self._current_channel = i2c_channel
			print("DEBUG: Shifted to TCA9548A Channel {}".format(i2c_channel))

