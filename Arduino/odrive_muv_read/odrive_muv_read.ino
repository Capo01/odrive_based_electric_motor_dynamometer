
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// Serial to the ODrive
SoftwareSerial odrive_serial(19, 18); //RX (ODrive TX), TX (ODrive RX)
// Note: you must also connect GND on ODrive to GND on Arduino!

// ODrive object
ODriveArduino odrive(odrive_serial);

void setup() {
  // ODrive uses 115200 baud
  odrive_serial.begin(115200);
  odrive_serial << "w axis0.muv1 " << 110.0f << '\n';
}

void loop() {}
