#!/usr/bin/env python3

"""
Odrive electric motor dynamometer test procedure.This file states what 
tests will be conducted their parameters.
https://github.com/Capo01/MotorDyno
R. Parsons

Example:
	odrive_startup()
    no_load_max_speed(500, 100)
    odrive_shutdown()
"""

# local source
from dyno_functions import *

# test procedure
odrive_startup()
report_motor_parameters()
write_values(measure_values())
#no_load_speed_test(odrive)			# Accepted arguments are either absorber_motor_odrive or test_motor_odrive
#motor_controller_loss_test(odrive)	# Accepted arguments are either absorber_motor_odrive or test_motor_odrive
odrive_shutdown()