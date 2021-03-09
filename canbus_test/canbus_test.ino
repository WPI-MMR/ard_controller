/**
 * Experimentation script using https://github.com/collin80/due_can/blob/master/examples/CAN_AutoBaud/CAN_AutoBaud.ino
 * as guidance.
 * 
 * ============= SEQUENCE THAT NEEDS TO BE IMPLEMENTED IN CAN PROTOCOL
 * 1) odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE (startup)
 * 2) odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL (startup)
 * 3) odrv0.axis0.controller.input_pos = <float> (repeat)
 * 
 * documentation of ODrive CAN protocol -> https://docs.odriverobotics.com/can-protocol
 * example of data frame for pos control -> https://discourse.odriverobotics.com/t/can-interface-available-for-testing/1448/8?u=andrew_103
 * 
 */

#include "variant.h"
#include <due_can.h>
// #include <DueTimer.h>

void setup() {
  Serial.begin(115200);
  Can0.begin(CAN_BPS_250K);

  Can0.watchFor();
  // for (int filter = 0; filter < 3; filter++) {
  //   Can0.setRXFilter(filter, 0, 0, true);
  // }
  // for (int filter = 3; filter < 15; filter++) {
  //   Can0.setRXFilter(filter, 0, 0, false);
  // }

  // while (Can0.available() == 0) {};
}

void print_frame(CAN_FRAME &frame) {
  Serial.print("ID: 0x");
  Serial.print(frame.id, HEX);
  Serial.print(" FID: 0x");
  Serial.print(frame.fid, HEX);
  Serial.print(" RTR: 0x");
  Serial.print(frame.rtr, HEX);
  Serial.print(" PRI: 0x");
  Serial.print(frame.priority, HEX);
  Serial.print(" EXT: 0x");
  Serial.print(frame.extended, HEX);
  Serial.print(" Len: ");
  Serial.print(frame.length);
  Serial.print(" Data: 0x");
  for (int count = 0; count < frame.length; count++) {
    Serial.print(frame.data.bytes[count], HEX);
    Serial.print(" ");
  }
  Serial.print("\r\n");  
}

void rx_serial_command() {
  if (Serial.available() > 0) {
    Serial.println("ping");
    String input = Serial.readString();
    CAN_FRAME outgoing;

    outgoing.id = (0x00 << 5) + 0x00C;
    outgoing.length = 8;
    outgoing.priority = 0;
    outgoing.rtr = 0x00;
    outgoing.extended = false;
    outgoing.data.byte[0] = (uint32_t)50;
    outgoing.data.byte[1] = (uint32_t)50 >> 8;
    outgoing.data.byte[2] = (uint32_t)50 >> 16;
    outgoing.data.byte[3] = (uint32_t)50 >> 24;
    outgoing.data.byte[4] = 0;
    outgoing.data.byte[5] = 0;
    outgoing.data.byte[6] = 0;
    outgoing.data.byte[7] = 0;
    Serial.println(Can0.sendFrame(outgoing));
  }
}

void loop() {
  rx_serial_command();
  if (Can0.available() > 0) {
    CAN_FRAME incoming;
    Can0.read(incoming);
    print_frame(incoming);
  }
}
