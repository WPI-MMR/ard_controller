#include <DueTimer.h>

#define PREAMBLE_LENGTH 4
#define DATA_BYTE_LENGTH 2

enum SerialReadState {
  INIT,
  PREAMBLE,
  
};

SerialReadState sr_state = INIT;

struct JointAngleStruct {
  int left_hip;
  int left_knee;
  int right_hip;
};

JointAngleStruct temporary_packet_data;
JointAngleStruct validated_packet_data;

bool write_flag = false;

void rx_processor() {
  static int preamble_counter = PREAMBLE_LENGTH;   // Number of ints in the packet preamble
  static int calculated_checksum = 0xFF; // We have to keep a running checksum of incoming data to verify validity
  int received_data; // The current int read from the RPi

  if (Serial1.available() > 0) {
    received_data = Serial1.read();
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);

  Timer1.attachInterrupt(serial_ISR).setFrequency(1).start();
}

void loop() {
  int data;
  if (Serial1.available() > 0) {
    data = Serial1.read();
    Serial.println("pong");
  }

  if (write_flag) {
    Serial.println("ping");
    write_flag = false;
  }
}

void serial_ISR() {
  Serial1.write((byte)255);
  write_flag = true;
}
