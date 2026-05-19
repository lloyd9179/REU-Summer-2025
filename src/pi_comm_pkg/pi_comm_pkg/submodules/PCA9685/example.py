#!/usr/bin/env python3

import PCA9685 as PCA
import time

pca = PCA.PCA9685()
"""
Defaults to address 0x40, oscilator_freq of 25MHz and servo_freq of 50Hz
"""

pca.set_oscillator_freq(27100000) # This value must be calibrated for each board

"""
Coverts the desired position of the servo in radians to a PWM signal
Only works for the specific type of servo used
"""
def servo_rad2pwm(radians):
	# For DS 3218 Servo
	MIN_USEC = 500.0
	MAX_USEC = 2500.0
	RANGE_RAD = 4.7124
	usec_per_rad = (MAX_USEC-MIN_USEC)/RANGE_RAD
	usec = ((MAX_USEC-MIN_USEC)/2.0) + MIN_USEC + radians*usec_per_rad
	usec = max(MIN_USEC, min(MAX_USEC, usec))
	pwm = int(round((usec/20000.0)*4095)) # 50hz -> 20ms pulse width
	return pwm


"""
Cycle a servo in channel 0 from -90 to 90 degrees
Actual output depends on servo type and its settings.
"""
while(1):
    #pca.set_pwm(14, 0, servo_rad2pwm(0))
    #pca.set_pwm(15, 0, servo_rad2pwm(.25))
    for itr in [0,0.7,0,-0.7]:
        value = servo_rad2pwm(itr)
        pca.set_pwm(14,0,value)
        time.sleep(2)

