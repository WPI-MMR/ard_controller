#include <ODriveArduino.h>
#include <HardwareSerial.h>

HardwareSerial& odrive_serial = Serial2;

ODriveArduino odrive(odrive_serial);

void setup() {
  odrive_serial.begin(115200);
  Serial.begin(115200);

  while (!Serial) ; // waits for serial monitor to open

}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'c') { // run calibration
      Serial.println("Calibrating...");

      int requested_state;
      
      requested_state = ODriveArduino::AXIS_STATE_FULL_CALIBRATION_SEQUENCE;
      if (!odrive.run_state(0, requested_state, true)) return;

      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL; // not triggering for some reason
      if (!odrive.run_state(0, requested_state, false)) return;
    }

    if (c == 'l') { // enter closed loop control mode
      Serial.println("Entering closed loop control...");

      int requested_state;

      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL; // successfully triggers
      if (!odrive.run_state(0, requested_state, false)) return;
    }

    if (c == 'm') { // move to specified motor position
      Serial.println("Moving to position 10...");
      odrive.SetPosition(0, 10.0f);
    }

    if (c == 'h') { // move to home position (0)
      Serial.println("Moving to position 0...");
      odrive.SetPosition(0, 0.0f);
    }

    if (c == 'e') { // get encoder position estimates
      Serial.println("Getting encoder position estimates...");
      odrive_serial.write("r axis0.encoder.pos_estimate\n");
      Serial.println(odrive.readFloat());
    }
  }

}
