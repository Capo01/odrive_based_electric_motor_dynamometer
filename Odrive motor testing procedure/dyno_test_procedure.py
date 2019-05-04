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

# Start up. Must be run to find odrives.
odrive_startup()

###### Place your desired dest procedures here #######
#report_motor_parameters()
#no_load_speed_test()
#motor_controller_loss_test()
test_motor_efficiency_map()
odrive_shutdown()