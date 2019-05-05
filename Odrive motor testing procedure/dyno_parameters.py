#!/usr/bin/env python3

"""
Odrive electric motor dynamometer parameters. Contains parameters of the 
motor to be tested, the absorber motor (if used) and the test reports. 
https://github.com/Capo01/MotorDyno
R. Parsons

TODO
-> add Ibus to test list.
-> Motor efficiency test function
-> Have temp_check() check that a thermistor is properly connected.

"""
# General
num_odrives                             = 2                 # Number of odrive that you are using
test_motor_axis                         = 1                 # The axis which thest motor is connected, either 0 or 1
absorber_motor_axis                     = 0                 # The axis which the absorber motor is connected, either 0 or 1
calibrate_on_run                        = True              # Motors will calibrate when test started if True
idle_on_finish                          = True              # Motors will become idle at end of all tests
max_safe_temp                           = 80                # Maximum allowable temperature for motor and MOSFETs [Deg C]
ignore_temp_warning                     = False             # Will ignore high temperature readings and continue testing.
emergency_stop_flag                     = False             # Set true if motor or controller temp goes outside set limits.
shutdown_ramp_speed                     = 1E6               # Speed to slow down at end of test [counts/s/s]

# Test data acquisition
text_file_name                          = 'odrive_data.csv' # Name of output text file which will contain the test data
text_file_name_motor_param              = 'motor_param.csv' # Contains each motors fixed inductance and resistance values
file_header_flag                        = True              # Set False to exclude the file header (name and unit at start of file)
erase_file_on_startup_flag              = True              # Set True to erase text file at the start of each run or between tests
stabilise_time                          = 1                 # Time elapsed after setting parameters before measuring values [s]
num_readings                            = 10                # Number of readings to be averaged
sleep_time                              = 0.01              # Wait time between readings. Arduino Mega only outputs a reading every 10 ms [s]

# Motor thermistor values (NTC 10k 1% 3435)
voltage_reference                       = 3.3               # voltage used for thermistor readings [V]
thermistor_nominal_val                  = 1E4               # Thermistor resistance [Ohm]
thermistor_coefficient                  = 3435              # Thermistor coefficient
thermistor_r0                           = 25                # Temperature at which thermistor has its nominal resistance [deg C]
series_resistor_val                     = 1E4               # Series resistor used in thermistor voltage divider [Ohm]

# No load speed test
no_load_motor_to_test                   = 2                 # 1 = Test motor, 2 = absorber
no_load_start_motor_speed               = 1000              # Motor speed to start test [RPM]
no_load_max_motor_speed                 = 1500              # Maximum safe motor speed to stop at if reached [RPM]
no_load_speed_step                      = 250               # Step size between measurements [RPM]
no_load_max_vel_error                   = 5                 # Maximum allowable error between motor commanded speed and measured speed [%]  

# Motor controller loss test
loss_est_motor_to_test                  = 2                 # 1 = Test motor, 2 = absorber
loss_est_num_steps                      = 5                 # Number of steps to increment motor current and take a reading
loss_est_current_step                   = 5                 # Current size between readings [A]
loss_est_max_motor_temp                 = 30                # Maximum motor temperature that a reading will be made at.

# Test motor efficiency test
efficiency_speed_step                   = 100               # Test motor speed incriment size [RPM]
efficiency_speed_max                    = 1000               # Test motor maximum speed to be tested [RPM]
efficiency_current_step                 = 5                 # Absorber current incriment size. Determines torque steps [A]
efficiency_current_max                  = 25                # Absorber current maximum. Determines maximum torque [A]

# Absorber motor Odrive
class Absorber_motor:
    serial_number                       = 35795122466891    # Unique serial number of odrive to be used. Can be found using Odrivetool 'odrv0.serial_number'
    brake_resistance                    = 0.5               # Default = 0.5 [Ohm] Resistance of the brake resistor connected to test motor odrive
    calibration_current                 = 20                # Default = 10 [A]  Current used during motor calibration
    current_lim                         = 60                # Default = 10 [A] Peak current supplied to the motor when in use
    pole_pairs                          = 7                 # Default = 7 [pole pairs]  Number of pole pairs. Count number of magnets and divide by 2
    vel_limit                           = 20000             # Default = 20000 [counts/s] Maximum motor speed    
    encoder_cpr                         = 8192              # Default = 8192 [CPR] Encoder counts per rotation
    vel_ramp_rate                       = 30000             # Default = 0 [CPR/S/S] Maximum speed change when in ramp mode      

# Test motor Odrive
class Test_motor:
    serial_number                       = 53237081453109    # Unique serial number of odrive to be used. Can be found using Odrivetool 'odrv0.serial_number'        
    brake_resistance                    = 0.47              # Default = 0.5 [Ohm] Resistance of the brake resistor connected to test motor odrive
    calibration_current                 = 20                # Default = 10 [A]  Current used during motor calibration
    current_lim                         = 60                # Default = 10 [A] Peak current supplied to the motor when in use
    pole_pairs                          = 7                 # Default = 7 [pole pairs]  Number of pole pairs. Count number of magnets and divide by 2
    vel_limit                           = 20000             # Default = 20000 [counts/s] Maximum motor speed
    encoder_cpr                         = 8192              # Default = 8192 [CPR] Encoder counts per rotation
    vel_ramp_rate                       = 30000             # Default = 0 [CPR/S/S] Maximum speed change when in ramp mode   

"""
Test motor and motor controller measurement list.
Each individual measurements has a name, unit and location which is placed into a dictionary.
Comment out those measurements you don't need.
More parameters can be added to the list as needed using the example below.

Example:
    
    'name'      :   'Input voltage', <---- Name printed in header of text file
    'unit'      :   'V',             <---- Unit printed in header of text file
    'location'  :   'absorber_motor_odrive.vbus_voltage', <--- Value to measure from odrive
"""

measurement_list = [
{
    'name'      :   'Input voltage',
    'unit'      :   'V',
    'location'  :   '(absorber_motor_odrive.vbus_voltage)',
},
{
    'name'      :   'Input current',
    'unit'      :   'mA',
    'location'  :   '(absorber_motor_odrive.axis0.muv2)',
},
{
    'name'      :   'Input Power',
    'unit'      :   'W',
    'location'  :   '(absorber_motor_odrive.vbus_voltage * absorber_motor_odrive.axis0.muv2 * 0.001)',
},
{
    'name'      :   'Test Motor Temperature',
    'unit'      :   'Deg C',
    'location'  :   '(motor_temp(test_motor_odrive))',
},
{
    'name'      :   'Test motor FET temp',
    'unit'      :   'Deg C',
    'location'  :   '(test_motor_odrive.axis0.motor.get_inverter_temp())',
},
{
    'name'      :   'Test motor q-axis current',
    'unit'      :   'A',
    'location'  :   '(test_motor_odrive.axis0.motor.current_control.Iq_measured)',
},
{
    'name'      :   'Test motor d-axis current',
    'unit'      :   'A',
    'location'  :   '(test_motor_odrive.axis0.motor.current_control.Id_measured)',
},
{
    'name'      :   'Test motor Ibus',
    'unit'      :   'A',
    'location'  :   '(test_motor_odrive.axis0.motor.current_control.Ibus)',
},
{
    'name'      :   'Test motor Ical',
    'unit'      :   'A',
    'location'  :   '(test_motor_odrive.axis0.motor.config.calibration_current)',
},
{
    'name'      :   'Output Torque',
    'unit'      :   'N.mm',
    'location'  :   'absorber_motor_odrive.axis0.muv1',
},
{
    'name'      :   'Output Speed',
    'unit'      :   'rad.s',
    'location'  :   '((absorber_motor_odrive.axis0.encoder.vel_estimate / 8192) * 2 * math.pi)',
},
{
    'name'      :   'Output Speed set-point',
    'unit'      :   '%',
    'location'  :   '(absorber_motor_odrive.axis0.controller.vel_setpoint)',
},
{
    'name'      :   'Output Power',
    'unit'      :   'W',
    'location'  :   '((absorber_motor_odrive.axis0.encoder.vel_estimate / 8192) * 2 * math.pi) * (absorber_motor_odrive.axis0.muv1) * 0.001',
},
# {
#     'name'      :   'Output Efficiency',
#     'unit'      :   '%',
#     'location'  :   '((((absorber_motor_odrive.axis0.encoder.vel_estimate / 8192) * 2 * math.pi) * (absorber_motor_odrive.axis0.muv1) * 0.001) / (absorber_motor_odrive.vbus_voltage * absorber_motor_odrive.axis0.muv2) * 100)',
# },
{
    'name'      :   'Absorber Motor Temperature',
    'unit'      :   'Deg C',
    'location'  :   '(motor_temp(absorber_motor_odrive))',
},
{
    'name'      :   'Absorber motor FET temp',
    'unit'      :   'Deg C',
    'location'  :   '(absorber_motor_odrive.axis0.motor.get_inverter_temp())',
},
{
    'name'      :   'Absorber q-axis current',
    'unit'      :   'A',
    'location'  :   '(absorber_motor_odrive.axis0.motor.current_control.Iq_measured)',
},
{
    'name'      :   'Absorber d-axis current',
    'unit'      :   'A',
    'location'  :   '(absorber_motor_odrive.axis0.motor.current_control.Id_measured)',
},
{
    'name'      :   'Absorber Ibus',
    'unit'      :   'A',
    'location'  :   '(absorber_motor_odrive.axis0.motor.current_control.Ibus)',
},
{
    'name'      :   'Absorber motor Ical',
    'unit'      :   'A',
    'location'  :   '(absorber_motor_odrive.axis0.motor.config.calibration_current)',
}
]