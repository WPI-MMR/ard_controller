/**
 * Experimentation script using https://github.com/collin80/due_can/blob/master/examples/CAN_AutoBaud/CAN_AutoBaud.ino
 * as guidance.
 */

#include <due_can.h>
#include <DueTimer.h>

void setup() {
  Serial.begin(115200);
  Can0.begin(CAN_BPS_250K);

  for (int filter = 0; filter < 3; filter++) {
    Can0.setRXFilter(filter, 0, 0, true);
  }
  for (int filter = 3; filter < 15; filter++) {
    Can0.setRXFilter(filter, 0, 0, false);
  }
}

void print_frame(CAN_FRAME &frame) {
   Serial.print("ID: 0x");
   Serial.print(frame.id, HEX);
   Serial.print(" Len: ");
   Serial.print(frame.length);
   Serial.print(" Data: 0x");
   for (int count = 0; count < frame.length; count++) {
       Serial.print(frame.data.bytes[count], HEX);
       Serial.print(" ");
   }
   Serial.print("\r\n");  
}

void loop() {
  if (Can0.available() > 0) {
    CAN_FRAME incoming;
    Can0.read(incoming);
    print_frame(incoming);
  }
}
