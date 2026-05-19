#!/usr/bin/env python3
"""
PCA9685.py
Function: A simple class to operate an PCA9685 I2C servo controller on an RPi with Python
PCA9685 is a 16-Channel, 12-bit PWM/Servo Driver
Author: Benjamin Walt
Date: 12/30/2022
Version: 0.1
Copyright (c) Benjamin Thomas Walt
Licensed under the MIT license.
"""


import smbus
from time import sleep


_PCA9685_MODE1 = 0x00
_PCA9685_MODE1_RESTART = 0x80
_PCA9685_PRE_SCALE = 0xFE
_PCA9685_OSCILLATOR_FREQUENCY = 25000000 # Default - Need to calibrate
_PCA9685_SERVO_FREQUENCY = 50 # Standard for most servos
_PCA9685_PRE_SCALE_MIN = 3  # minimum prescale value
_PCA9685_PRE_SCALE_MAX = 255 # maximum prescale value

# This is the start point for all the LED registers
# There are 4 registers per LED
# ON Low - 8 bits
# ON High - 4 bits
# OFF Low - 8 bits
# OFF High - 4 bits
_PCA9685_LED0_ON_L = 0x06


class PCA9685:
	"""A class used to set up and control the PCA9685 a 16-Channel, 12-bit PWM/Servo Driver"""
	def __init__(self, address = 0x40):
		# ~ self._bus = smbus.SMBus(1) # Channel = 1
		self._address = address
		self._oscillator_freq = _PCA9685_OSCILLATOR_FREQUENCY
		# ~ self._write_reg(_PCA9685_MODE1, _PCA9685_MODE1_RESTART)
		# ~ sleep(0.1)
		# ~ self.set_oscillator_freq(self._oscillator_freq) #Also sets servo frequency
		# ~ self._set_sleep_bit(1) # Sleep until needed.
		print("DEBUG: PCA9685 started in debug mode")
		
	def _write_reg(self, reg, value):
		"""Write a byte of data to a given register"""
		# ~ self._bus.write_byte_data(self._address, reg, value)
		pass

	def _read_reg(self, reg):
		"""Read a byte of data from a given register"""
		# ~ return self._bus.read_byte_data(self._address, reg)
		pass
	
	# ~ def _set_sleep_bit(self, value):
		# ~ """Sets the sleep bit of the mode 1 register without changing the other bits"""
		# ~ sleep_bit = 4 # fifth bit
		# ~ current_state = self._read_reg(_PCA9685_MODE1)
		# ~ if(value == 1):
			# ~ new_state = current_state | (0x01 << sleep_bit)
		# ~ else:
			# ~ new_state = current_state & ~(0x01 << sleep_bit)
		# ~ self._write_reg(_PCA9685_MODE1, new_state)
		# ~ sleep(0.0005) # requires time to wake up
	
	def set_servo_freq(self, frequency = _PCA9685_SERVO_FREQUENCY):
		"""Sets the frequency at which the PWM operates."""
		"""
		50hz is common for servos.
		It means that the full pulse is 20ms and the duty-cycle is a fraction of that.
		0 is 0ms (0% duty-cycle)
		2048 is 10ms (50% duty-cycle)
		4095 is 20ms (100% duty-cycle)
		Note: Servo only uses a small band in the duty-cycle e.g 500-2500us
		"""
		prescalevel = round((self._oscillator_freq / (frequency * 4096.0))) - 1
		prescalevel = max(_PCA9685_PRE_SCALE_MIN, min(_PCA9685_PRE_SCALE_MAX, prescalevel))
		prescale = int(prescalevel)
		self._set_sleep_bit(1) # go to sleep - Need to sleep to set prescale
		self._write_reg(_PCA9685_PRE_SCALE, prescale) # set the prescaler
		self._set_sleep_bit(0) # Wake up
		sleep(0.1)
		
	def set_pwm(self, channel, on, off):
		"""Sets the on and off point for the pwm signal. on is typically just 0 and off is a number less than 4096????"""
		# ~ self._set_sleep_bit(0) # Wake up
		# ~ self._bus.write_byte_data(self._address, _PCA9685_LED0_ON_L + 4*channel, on)
		# ~ self._bus.write_byte_data(self._address, _PCA9685_LED0_ON_L + 4*channel +1, on >> 8)
		# ~ self._bus.write_byte_data(self._address, _PCA9685_LED0_ON_L + 4*channel +2, (off & 0xFF))
		# ~ self._bus.write_byte_data(self._address, _PCA9685_LED0_ON_L + 4*channel +3, off >> 8)
		# ~ self._set_sleep_bit(1) # Sleep
		print("DEBUG: Setting PWM signal")
		
	def set_oscillator_freq(self, freq=_PCA9685_OSCILLATOR_FREQUENCY):
		""" Sets the oscillator frequency (not the servo frequency)."""
		# This will be in the MHz range - around 25Mhz
		self._oscillator_freq = freq
		self.set_servo_freq()
		
