#include <DueTimer.h>

bool write_flag = false;

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
