### odrive_based_electric_motor_dynamometer
An open source four-quadrant brushless electric motor dynamometer based on the [Odrive Robotics](https://odriverobotics.com/) motor controller.

**Project status**: On hold until the end of 2019 due to other commitments.

## Features
* Desinged to test motors from 50 to 2000 W over 0 to 7500 RPM with a peak breaking torque of 3.5 N.m
* Different sized motors can be accomadated through the use of different absorber motors
* Can map motor efficiency and power loss for both motoring and generating (four quadrant operation)
* Can measure static torque and therefore torque constant and motor constant.
* Can measure phase inductance and resistance at room temperature and at elevated temperature
* Can measure no load maximum speed and power draw
* Capable of seperating the power loss in the motor and motor controller
* Uses low cost components for current sensing (current shuts) and torque sensing (load cell)
* Breaking energy recovered and supplied to test motor in a closed loop to reduce power supply requiremnets

## Design 
The motor under test is connected by a shaft coupler directly to a D6374 sized motor which is used as an absorber. The absorber motor provideds a breaking toque or driving torque when testing generating performance of the test motor. Torque is measured by a load cell which is attached to the frame of the absorber motor by a pivot arm. Power draw is estimated by current shunt and bus voltage which is monitored by a secondary microcontroller. All control is handeled by the Odrive motor controller using a python script which outputs a text file with all the measurment results.

See [this album](https://photos.app.goo.gl/ma1DWoY4Qa5PH14DA) for more images of the test bench.

## Preliminary results
