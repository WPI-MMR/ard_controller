#include <ODriveArduino.h>
#include <HardwareSerial.h>

HardwareSerial& odrive_serial = Serial1;

ODriveArduino odrive(odrive_serial);

void setup() {
  odrive_serial.begin(115200);
  Serial.begin(115200);

  while (!Serial) ; // waits for serial monitor to open

}

void loop() {
  if (Serial.available()) {
    
  }

}
