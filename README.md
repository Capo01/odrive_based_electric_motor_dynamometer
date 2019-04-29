# OpenDyno
A low cost open source four quadrant electric motor dynamometer based on the Odrive brushless motor controller.

Inline-style: 
![alt text](https://github.com/Capo01/odrive_based_electric_motor_dynamometer/Docs/IMG_20190330_130623.jpg "OpenDyno")

# Introduction
Very little reliable data is currently available on the performance and efficiency of hobbyist class brushless motors.
Four quadrant (forward motor, forward braking, reverse noting, reverse braking) Electric motor dynamometer allow 

OpenDyno aims to fill this knowledge gap by

OpenDyno 


# Hardware
* [Odrive brushless motor controller](https://odriverobotics.com/)
* [D6374 sized brushless Motor (150KV) or similar](https://odriverobotics.com/shop/odrive-custom-motor-d6374-150kv)
* [Arduino based microcontroller](https://www.arduino.cc/)
* [HX711 based load cell amplifer](https://www.sparkfun.com/products/13879)
* [5kg Load cell](https://www.sparkfun.com/products/14729)
* [100A 75mV current shunt resistor](https://www.banggood.com/100A-75mV-DC-Current-Shunt-Resistor-Panel-For-Analog-Meter-p-916882.html?rmmds=buy&cur_warehouse=CN)
* [24 bit ADC](https://www.banggood.com/ADS1256-24-Bit-8-Channel-ADC-AD-Module-High-Precision-ADC-Acquisition-Data-Acquisition-Card-p-1231051.html?utm_design=41&utm_source=emarsys&utm_medium=Shipoutinform171129&utm_campaign=trigger-emarsys&utm_content=Winna&sc_src=email_2671705&sc_eh=8eb87b081b346f0b1&sc_llid=12245521&sc_lid=104858042&sc_uid=TjcOYL9zCg&cur_warehouse=CN)











Arduino Mega 1280 used to capture load cell amplifer data (HX711)
Load cell force converted to torque and converted to a 500 Hz PWM signal
Due to the limited 8 bit resolution of the stock arduino library, an additional PWM library is used to increase this to 16bit.
library can be found here https://code.google.com/archive/p/arduino-pwm-frequency-library/downloads
Pin 11 used on arduino mega for PWM
12 bit resolution at 500Hz
The odrive range mapping is between 1000 and 2000 microseconds high-time
Major problem with noise in the signal. needs to be filtered out. 
Electricall isolating the loadcell from the motor mount help reduce most of the noise.

Pololu ACS714 Current sensor -30A to 30A https://www.pololu.com/product/1187
Signal read with arduino analog in with 1024 steps
converted to PWM at 500Hz 12bit output 
read with gpio pin 3
ended up not using due to noise and problems with getting a zero reading

Used a 20A 75mV current shunt with a sparkfun ADS1115 breakout board
Current shunt placed on low side of PSU power to odrive
x16 gain used (1bit = 0.0078125mV) with differential input





Youtube clip
https://youtu.be/YfqsjVh0d5g

