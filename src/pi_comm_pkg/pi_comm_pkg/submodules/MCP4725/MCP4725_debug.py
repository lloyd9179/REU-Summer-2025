#!/usr/bin/env python
"""
MCP4725.py
Function: A simple class to operate an MCP4725 DAC on an RPi with Python
Author: Benjamin Walt
Date: 9/29/2020
Purpose: SoftAgBot system integration project
Version: 0.1
This is a rework of Adafruit's software:
https://github.com/adafruit/Adafruit_MCP4725
https://www.adafruit.com/product/935
"""
import smbus

_DEVICE_ADDRESS = 0x62 #This depends on manufacturer, may need to make an input during init
# 0x62 is adafruit
# 0x60 is sparkfun
_REG_WRITE_DAC = 0x40

class MCP4725():
	
	def __init__(self, address):
		# ~ self._bus = smbus.SMBus(1) # Channel = 1
		self._address = address
		self.set_dac_voltage(0.0)
		print("DEBUG: MCP4725 started in debug mode")

	def _write_reg(self, reg, value):
		# ~ self._bus.write_i2c_block_data(self._address, reg, value)
		pass
		
	def set_dac_voltage(self, voltage):
		digital_val = int((voltage/5.0)*4095) # Create a value btween 0 and 4095
		digital_val = max(0, min(4095, digital_val))
		
		# Shift everything left by 4 bits and separate bytes
		upper = (digital_val & 0xff0) >> 4 # Upper data bits (D11.D10.D9.D8.D7.D6.D5.D4)
		lower = (digital_val & 0xf) << 4 # Lower data bits (D3.D2.D1.D0.x.x.x.x)
		msg = [upper, lower]
		# ~ self._write_reg(_REG_WRITE_DAC, msg)
		print("DEBUG: MCP4725 setting voltage to {}".format(voltage))

