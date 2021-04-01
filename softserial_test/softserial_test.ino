/**
 * NOTE: Software Serial should only be used in applications in which there are not enough
 * hardware serial ports on the microcontroller or SBC being used
 */

#include <ODriveArduino.h>
#include <soft_uart.h>

using namespace soft_uart;
using namespace soft_uart::arduino_due;

#define RX_PIN 10
#define TX_PIN 11
#define SOFT_UART_BIT_RATE 115200 // 57600 38400 1200 19200 9600 115200 300
#define RX_BUF_LENGTH 256 // software serial port's reception buffer length
#define TX_BUF_LENGTH 256 // software serial port's transmision buffer length

serial_tc4_declaration(RX_BUF_LENGTH,TX_BUF_LENGTH);
auto& odrive_serial = serial_tc4; // serial_tc4_t& serial_obj=serial_tc4;

ODriveArduino odrive(odrive_serial);

void setup() {
  Serial.begin(115200);

  odrive_serial.begin(
    RX_PIN,
    TX_PIN,
    SOFT_UART_BIT_RATE,
    soft_uart::data_bit_codes::EIGHT_BITS,
    soft_uart::parity_codes::NO_PARITY,
    soft_uart::stop_bit_codes::ONE_STOP_BIT
  );

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
    }

    if (c == 'l') { // enter closed loop control mode
      Serial.println("Entering closed loop control...");

      int requested_state;
      String message;

      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL; // successfully triggers
      if (!odrive.run_state(0, requested_state, false)) return;
    }

    if (c == 'm') { // move to specified motor position
      Serial.println("Moving to position 10...");
      // The software serial library used in this sketch has an issue that
      // prevents the "p" command from being used to communicate with the ODrive

      // odrive.SetPosition(0, 10.0f); // doesn't work because uses "p" command
      // odrive_serial.write("p 0 10 0 0\n"); // doesn't work with floats
      odrive_serial.write("w axis0.controller.input_pos 10.0\n");
    }

    if (c == 'h') { // move to home position (0)
      Serial.println("Moving to position 0...");
      odrive_serial.write("w axis0.controller.input_pos 0.0\n");
    }

    if (c == 'e') { // get encoder position estimates
      Serial.println("Getting encoder position estimates...");
      odrive_serial.write("r axis0.encoder.pos_estimate\n");
      Serial.println(odrive.readFloat());
    }
  }
}
