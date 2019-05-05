#!/usr/bin/env python3

"""
Odrive electric motor dynamometer function module. 
https://github.com/Capo01/MotorDyno
R. Parsons

Example:
    odrive_startup()
    no_load_max_speed(500, 100)
    odrive_shutdown()
"""

# 3rd party packages
from __future__ import print_function
import odrive
from odrive.enums import *

# standard libraries
import math
import time
import statistics
import datetime
import matplotlib.pyplot as plt

# local sources
from dyno_parameters import *

def rpm_to_cpr(x, odrive):
    
    """
    Converts motor RPM to counts/s for use by motor controller.
    """
    if odrive == absorber_motor_odrive:
        encoder_cpr = Absorber_motor.encoder_cpr
    if odrive == test_motor_odrive:
        encoder_cpr = Test_motor.encoder_cpr
   
    return (x / 60)* encoder_cpr;

def temp_check():
    """
    Compares motor and MOSFET temperatures to max_safe_temp.
    If exceeded, measurement stopped, motor speed reduced to zero, controller made idle and an error printed to console.
    """
    global emergency_stop_flag

    if ignore_temp_warning == True:
        return

    if all(x >= max_safe_temp for x in (
        motor_temp(absorber_motor_odrive),
        absorber_motor_odrive_axis.motor.get_inverter_temp(),
        motor_temp(test_motor_odrive),
        test_motor_odrive_axis.motor.get_inverter_temp()
    )):
        emergency_stop_flag = True
        print('Warning: Maximum safe temperature (', max_safe_temp, ' Deg C) exceeded.')
        print('Absorber Motor Temperature (Deg C): ', motor_temp(absorber_motor_odrive) )
        print('Absorber Odrive MOSFET Temperature (Deg C): ', absorber_motor_odrive_axis.motor.get_inverter_temp())
        print('Test Motor Temperature (Deg C): ', motor_temp(test_motor_odrive) )
        print('Test Odrive MOSFET Temperature (Deg C): ', test_motor_odrive_axis.motor.get_inverter_temp())
        odrive_shutdown()
        exit()

    return

def motor_temp(odrive):
    """
    Reads motor temperature thermistor which is connected to Odrive's analog input pins.
    Motors sold by Odrive have inbuilt thermistor thermally connected to motor windings.
    Note: get_adc_voltage() returns a voltage.
    """
    absorber_motor_reading  = absorber_motor_odrive.get_adc_voltage(3) # Read absorber motor ADC
    test_motor_reading      = absorber_motor_odrive.get_adc_voltage(4) # Read test motor ADC

    absorber_motor_temp_reading = ((1/ ((math.log10(((series_resistor_val / (voltage_reference  / absorber_motor_reading - 1))/thermistor_nominal_val))/thermistor_coefficient) + 1.0 / (25 + 273.15))) - 273.15)
    test_motor_temp_reading = ((1/ ((math.log10(((series_resistor_val / (voltage_reference  / test_motor_reading - 1))/thermistor_nominal_val))/thermistor_coefficient) + 1.0 / (25 + 273.15))) - 273.15)
    
    # print('Absorber Temp (Deg C): ', absorber_motor_temp_reading)
    # print('Test Temp (Deg C): ',test_motor_temp_reading)

    if odrive == absorber_motor_odrive:
        return absorber_motor_temp_reading;

    if odrive == test_motor_odrive:
        return test_motor_temp_reading;

    print('Motor temp error: incorrect motor name entered.')
    return;

def odrive_startup():
    """
    Connects to Odrive motor controllers and assigns them to be either test motor or absorber depending
    in their serial number. Assigns motor, encoder and controller parameters as described in dyno_parameters.py
    """
    # Find odrives all required odrives
    global absorber_motor_odrive
    global test_motor_odrive
    global absorber_motor_odrive_axis
    global test_motor_odrive_axis

    print('### Beginning Odrive startup ###')
    print('Looking for', num_odrives, 'Odrives...')

    odrives = odrive.find_any(timeout = 30, find_multiple=num_odrives)
    num_odrive_found = len(odrives)
    
    if num_odrive_found != num_odrives:
        if odrives[0].serial_number == Test_motor().serial_number:
            print(num_odrive_found, 'Odrives found. Please connect absorber Odrive')
        else:
            print(num_odrive_found, 'Odrives found. Please connect test Odrive')
        exit()

    print('Found', num_odrive_found, 'Odrives.')
    
    
    # Assign odrives based on their serial numbers.
    odrives_assigned = 0

    for odrv in odrives:
        if odrv.serial_number == Test_motor().serial_number:
            test_motor_odrive = odrv
            if test_motor_axis == 0:
                test_motor_odrive_axis = test_motor_odrive.axis0
            else:
                test_motor_odrive_axis = test_motor_odrive.axis1          
            odrives_assigned += 1
            print('Test motor assigned to serial number', test_motor_odrive.serial_number)
        if odrv.serial_number == Absorber_motor().serial_number:
            absorber_motor_odrive = odrv
            if absorber_motor_axis == 0:
                absorber_motor_odrive_axis = absorber_motor_odrive.axis0
            else:
                absorber_motor_odrive_axis = absorber_motor_odrive.axis1  
            odrives_assigned += 1
            print('Absorber motor assigned to serial number', absorber_motor_odrive.serial_number)
    
    if odrives_assigned != num_odrives:
        print('Listed odrives serial numbers do not match those in py')
        exit()

    # Set Odrive parameters as listed in dyno_parameters.py
    print('Setting parameters...')
    absorber_motor_odrive.config.brake_resistance = Absorber_motor().brake_resistance
    absorber_motor_odrive_axis.motor.config.calibration_current = Absorber_motor().calibration_current
    absorber_motor_odrive_axis.motor.config.current_lim = Absorber_motor().current_lim
    absorber_motor_odrive_axis.motor.config.pole_pairs = Absorber_motor().pole_pairs
    absorber_motor_odrive_axis.controller.config.vel_limit = Absorber_motor().vel_limit

    test_motor_odrive.config.brake_resistance = Test_motor().brake_resistance
    test_motor_odrive_axis.motor.config.calibration_current = Test_motor().calibration_current
    test_motor_odrive_axis.motor.config.current_lim = Test_motor().current_lim
    test_motor_odrive_axis.motor.config.pole_pairs = Test_motor().pole_pairs
    test_motor_odrive_axis.controller.config.vel_limit = Test_motor().vel_limit
    
    print('Setting parameters complete.')

    # Calibrate motors if desired.
    if calibrate_on_run == True:
        # Calibrate absorber motor and wait for it to finish
        print("Calibrating absorber motor...")
        absorber_motor_odrive_axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while absorber_motor_odrive_axis.current_state != AXIS_STATE_IDLE:
            time.sleep(0.5)
        print("Calibrating absorber motor complete.")

        # Calibrate test motor and wait for it to finish
        print("Calibrating test motor...")
        test_motor_odrive_axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while test_motor_odrive_axis.current_state != AXIS_STATE_IDLE:
            time.sleep(0.5)
        print("Calibrating absorber motor complete.")

    print("Setting closed loop control...")
    absorber_motor_odrive_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.5) # Prevent both motors coming active at once.
    test_motor_odrive_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    time.sleep(0.5)
    print("Now in closed loop control")

    print('### Odrive startup complete ###')

def odrive_shutdown():
    
    """
    Stops motor and end of test, sets motor to positional control mode or idle.
    """
    print("### Shutting down odrives ###")

    #Absorber motor and test motor shutdown
    absorber_motor_odrive_axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    test_motor_odrive_axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    absorber_motor_odrive_axis.controller.vel_ramp_enable = True
    test_motor_odrive_axis.controller.vel_ramp_enable = True
    absorber_motor_odrive_axis.controller.config.vel_ramp_rate = shutdown_ramp_speed
    test_motor_odrive_axis.controller.config.vel_ramp_rate = shutdown_ramp_speed
    absorber_motor_odrive_axis.controller.vel_ramp_target = 0
    test_motor_odrive_axis.controller.vel_ramp_target = 0

    # wait for motors to slow down.
    print("Slowing down motors...")
    while absorber_motor_odrive_axis.encoder.vel_estimate and test_motor_odrive_axis.encoder.vel_estimate != 0:
        time.sleep(0.5)
    
    print("Motors stopped")
    absorber_motor_odrive_axis.controller.vel_ramp_enable = False
    test_motor_odrive_axis.controller.vel_ramp_enable = False

    # Prevent motors being set to position zero at full speed when set to position control mode.
    print("Setting to position control mode")
    absorber_motor_odrive_axis.controller.pos_setpoint = absorber_motor_odrive_axis.encoder.pos_estimate 
    test_motor_odrive_axis.controller.pos_setpoint = absorber_motor_odrive_axis.encoder.pos_estimate 
    absorber_motor_odrive_axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    test_motor_odrive_axis.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    
    if idle_on_finish == True or emergency_stop_flag == True:
        # Calibrate motor and wait for it to finish
        absorber_motor_odrive_axis.requested_state = AXIS_STATE_IDLE
        test_motor_odrive_axis.requested_state = AXIS_STATE_IDLE

    print("### Shut down complete ###")

def measure_values():
    """
    Measures the values outlined in measurement_list.
    Takes multiple measurements as determined by num_readings.
    Averages the measurements and returns the data as a list.
    
    """
    data_list = []   
       
    # Measurement data to be stored in a new empty dictionary key called 'values'
    for i in range(len(measurement_list)):
        measurement_list[i]['value'] = []
    
    # Multiple measurements taken for averaging as per num_readings
    for x in range(num_readings):
        for i in range(len(measurement_list)):
                location = measurement_list[i]['location']
                measurement_list[i]['value'].append(eval(location))
                time.sleep(sleep_time) # odrive only outputs a reading every 10 ms
                
                    
    # Mean taken of measurements to filter out noise.
    for i in range(len(measurement_list)):
                data_list.append(statistics.mean(measurement_list[i]['value']))

    return data_list;

def write_values(data):
    """
    Takes measured data and writes it to a text file created in the same dir as this script.
    Will include header data if specified.
    
    """
    global file_header_flag
    global erase_file_on_startup_flag
    name_list = []
    unit_list = []
    data_list = data
    
    # Add the measurement names and units to a list for the csv header.
    for i in range(len(measurement_list)):
        name_list.append(measurement_list[i]['name'])
        unit_list.append(measurement_list[i]['unit'])
       
    # Format the header data  
    with open(text_file_name,"a+") as text_file:
        name_list_formatted = ', '.join(str(e) for e in name_list) + '\n'
        unit_list_formatted = ', '.join(str(e) for e in unit_list) + '\n'
        data_list_formatted = ', '.join(str(e) for e in data_list) + '\n'
                            
        # Erase the contents of the text file from previous runs (need '0' when using r+ )
        if erase_file_on_startup_flag == True:
            text_file.truncate(0)
            erase_file_on_startup_flag = False 

        # Write header data to text file only once.
        if file_header_flag == True:
            text_file.write(str(datetime.datetime.now()) + '\n') # Add date and time to text file.
            text_file.write(str(name_list_formatted))
            text_file.write(str(unit_list_formatted))
            file_header_flag = False
        
        # Write the data
        text_file.write(str(data_list_formatted))
  
def report_motor_parameters():
    """
    Tests motor resistance and inductance and prints it to a separate text file.
    """
    print("### Reporting motor parameters ###")
    temp_check() # Check all temperatures are safe.
    print("Testing absorber motor...")
    absorber_motor_odrive_axis.motor.config.calibration_current = Absorber_motor().calibration_current
    absorber_motor_odrive_axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    while absorber_motor_odrive_axis.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    print("Testing test motor...")
    test_motor_odrive_axis.motor.config.calibration_current = Test_motor().calibration_current
    test_motor_odrive_axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    while test_motor_odrive_axis.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)

    print("Testing complete...")   

    absorber_motor_resistance = absorber_motor_odrive_axis.motor.config.phase_resistance
    absorber_motor_inductance = absorber_motor_odrive_axis.motor.config.phase_inductance
    absorber_motor_temperature = motor_temp(absorber_motor_odrive)
    print('Absorber motor resistance (Ohm): ', absorber_motor_resistance)
    print('Absorber motor inductance (Henry): ', absorber_motor_inductance)
    print('Absorber motor Temperature (Deg C): ', absorber_motor_temperature)

    test_motor_resistance = test_motor_odrive_axis.motor.config.phase_resistance
    test_motor_inductance = test_motor_odrive_axis.motor.config.phase_inductance
    test_motor_temperature = motor_temp(test_motor_odrive)
    print('Test motor resistance (Ohm): ', test_motor_resistance)
    print('Test motor inductance (Henry): ', test_motor_inductance)
    print('Test motor temperature (Deg C): ', test_motor_temperature)

    print("Saving motor parameters to a text file...")

    with open(text_file_name_motor_param,"a+") as text_file:
        text_file.write(str(datetime.datetime.now()) + '\n') # Add date and time to text file.
        text_file.write('Absorber motor resistance (Ohm) ,' + str(absorber_motor_resistance) + '\n')
        text_file.write('Absorber motor inductance (Henry) ,' + str(absorber_motor_inductance) + '\n')
        text_file.write('Absorber motor temperature (Deg C) ,' + str(absorber_motor_temperature) + '\n')
        text_file.write('Test motor resistance (Ohm) ,' + str(test_motor_resistance) + '\n')
        text_file.write('Test motor inductance (Henry) ,' + str(test_motor_inductance) + '\n')
        text_file.write('Test motor temperature (Deg C) ,' + str(test_motor_temperature) + '\n')

    print("### Finished reporting motor parameters ###")
     
def no_load_speed_test():
    
    """
    Estimates maximum motor speed and motor + motor controller no-load power loss.
    Increments motor speed until allowable_speed_error is exceeded.
    Logs data for each no_load_speed_step
    """
    if no_load_motor_to_test == 1:
        odrive = test_motor_odrive
    if no_load_motor_to_test == 2:
        odrive = absorber_motor_odrive

    odrive.axis0.controller.config.vel_limit = rpm_to_cpr(no_load_max_motor_speed, odrive)
    odrive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL 
    odrive.axis0.controller.vel_setpoint = 0 # Stop motor if it isn't already stopped [counts/s]
    while odrive.axis0.encoder.vel_estimate != 0:
        time.sleep(0.1)

    print("Starting no-load speed test")
    print("Maximum allowable speed error (%): ", no_load_max_vel_error)
    print('Increment, Set Speed (RPM), vel_error (%), Input Power (W)') 
    
    vel_error = 0.0
    no_load_max_error = no_load_max_vel_error / 100
    i = no_load_start_motor_speed / no_load_speed_step

    # Increment speed by no_load_speed_step until the maximum motor speed is reached
    while vel_error < no_load_max_error:
        temp_check() # Check all temperatures are safe
        set_speed = i * no_load_speed_step # [counts/s]
        
        # Prevent motor speed becoming dangerously fast
        if set_speed > no_load_max_motor_speed:
            print("Maximum safe speed of ", no_load_max_motor_speed, ' reached.')
            break

        # Set motor speed, wait for things to stabilise
        odrive.axis0.controller.vel_setpoint = rpm_to_cpr(set_speed, odrive) # [counts/s]
        time.sleep(stabilise_time) # wait for speed to stabilise [s]

        # Estimate how close motor speed is to that requested and prevent divide by zero
        if set_speed > 0:
            vel_error = 1 - (odrive.axis0.encoder.vel_estimate / odrive.axis0.controller.vel_setpoint) # speed error [%]
        input_power = odrive.vbus_voltage * odrive.axis0.muv2 * 0.001
        print(i, ' ', set_speed, ' ' , vel_error * 100, ' ', input_power)
        write_values(measure_values())
        i += 1

    print("No-load speed test complete")
    print("Maximum motor speed = " , set_speed - no_load_speed_step) # Revert set speed to previous value 

def motor_controller_loss_test():
    """
    Estimates Odrive I^2R losses by incrementally increasing motor current.
    Motor phase current and reported motor resistance report_motor_parameters() gives motor power draw.
    Subtracting motor power draw from Odrive power draw (current shunt reading x Vbus) gives Odrive I^2R loss.
    """
    if loss_est_motor_to_test  == 1:
        odrive = test_motor_odrive
    if loss_est_motor_to_test  == 2:
        odrive = absorber_motor_odrive

    print('Starting motor controller loss test..')
    print('Step No., Calibration current (A), Motor temperature (DegC)')
    odrive.axis0.requested_state = AXIS_STATE_IDLE
    for i in range(loss_est_num_steps):
        temp_check() # Check all temperatures are safe.

        # If the motor becomes too hot then its resistance will be different to that measured at start-up.
        # A higher than expected motor resistance will lead to an over-estimate Odrive I^2R losses.
        while motor_temp(odrive) >= loss_est_max_motor_temp:
            time.sleep(2)
            print('Motor too hot. Motor Temp (DegC): ', motor_temp(odrive))

        odrive.axis0.motor.config.calibration_current = (i + 1) * loss_est_current_step # Can not calibrate with current = 0
        # Offset calibration used as a means for the motor to draw a large current without doing mechanical work.
        # Offset calibration also loads each phases evenly, resulting in even MOSFET and motor phase heating.
        
        x = 0
        odrive.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        while odrive.axis0.current_state != AXIS_STATE_IDLE: # wait for offset calibration to finish before incrementing current 
            if x == 0:
                print(i, ' ', odrive.axis0.motor.config.calibration_current, ' ', motor_temp(odrive))
                time.sleep(stabilise_time)
                write_values(measure_values())
                x = 1 # Write to file only once per calibration current setting
            time.sleep(0.5)

    absorber_motor_odrive_axis.motor.config.calibration_current = Absorber_motor.calibration_current # set calibration current back to default value.
    test_motor_odrive_axis.motor.config.calibration_current = Test_motor.calibration_current
    print('Motor controller loss test complete.')

def test_motor_efficiency_map():
    """
    Used to produce an efficiency map of the test motor.
    Test motor set in velocity control mode (with speed ramping) and its speed incremented.
    At each speed a fixed torque is placed on the test motor by the absorber.
    After full range of speeds tested, absorber motor torque incremented.
    Torque is incremented for a given speed rather than the other way around so that the motor
    is not commanded to a high torque for long periods, generating a lot of heat.
    """
    print('### Starting test motor efficiency mapping ###')

    print('Setting test specific parameters...')
    test_motor_odrive_axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    test_motor_odrive_axis.controller.config.vel_limit = rpm_to_cpr(efficiency_speed_max, test_motor_odrive) * 1.5 # 1.5x added to prevent ERROR_OVERSPEED due to load changes on the motor.
    test_motor_odrive_axis.controller.vel_ramp_enable = True

    absorber_motor_odrive_axis.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
    absorber_motor_odrive_axis.controller.config.vel_limit = rpm_to_cpr(efficiency_speed_max, test_motor_odrive) * 1.5 # 1.5x added to prevent ERROR_OVERSPEED due to load changes on the motor.
    absorber_motor_odrive_axis.motor.config.current_lim = efficiency_current_max
    
    print('Starting test motor efficiency test..')
    print('Step No., Test motor speed (RPM), Absorber motor current (A)')

    for i in range(int(efficiency_speed_max / efficiency_speed_step)):
        set_speed = rpm_to_cpr((i + 1) * efficiency_speed_step, test_motor_odrive) # +1 added so starting speed != 0
        test_motor_odrive_axis.controller.vel_ramp_target = set_speed
        # wait for set_speed to be reached
        wait_num = 0
        while test_motor_odrive_axis.encoder.vel_estimate <= (set_speed * 0.98):
            time.sleep(0.1)
            wait_num += 1
            if wait_num == 20: # Let user know if speed is not reached within 2 seconds.
                print('Waiting for test motor to reach set speed...')
                time.sleep(1)
                
        for j in range(int(efficiency_current_max / efficiency_current_step) + 1):
            temp_check() # Check all temperatures are safe.
            set_current = j * efficiency_current_step
            print(str(i) + '.' + str(j), int((set_speed / Test_motor.encoder_cpr) * 60), set_current)
            
            # slowly increase or decrease current_setpoint to allow time for test motor PID to adjust.
            current_difference = set_current - absorber_motor_odrive_axis.controller.current_setpoint
            while -1 > current_difference > 1: # Only need to increase current slowly if difference > 1 A
                if current_difference > 0:
                    absorber_motor_odrive_axis.controller.current_setpoint += 1
                    time.sleep(0.2)
                if current_difference < 0:
                    absorber_motor_odrive_axis.controller.current_setpoint -= 1
                    time.sleep(0.2)
                current_difference = set_current - absorber_motor_odrive_axis.controller.current_setpoint
            absorber_motor_odrive_axis.controller.current_setpoint = set_current # add the remainder, if any.

            time.sleep(stabilise_time)
            write_values(measure_values())
            
    print('### Test motor efficiency mapping complete ###')

# Error due to ERROR_CONTROL_DEADLINE_MISSED. See discord. Try increasing speed and setting same gains.
