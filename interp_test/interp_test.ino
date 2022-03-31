#include <ODriveArduino.h>
#include <HardwareSerial.h>
#include <math.h>

#define GEAR_RATIO 9
#define NUM_JOINTS 2
#define NUM_IMU_AXES 3
#define PREAMBLE_LENGTH 4
#define DATA_BYTE_LENGTH 2
#define DEADBAND 1

// Serial2, 3, 4, and 5 are built in variables defined by the Teensy board definition
// these lines simply rename the serial objects to represent the odrive they're tied to
HardwareSerial& odrv_rightarm_ser = Serial2; // 3R + 4R

ODriveArduino odrv_rightarm(odrv_rightarm_ser);

int cur_joint_pos[] = {
  0, 0
};
float cur_raw_pos[] = {
  0, 0
};

int L1 = 160;
int L2 = 173.5;

int t1[] = {
  0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 90
};
int t2[] = {
  0, 350, 340, 330, 320, 310, 300, 290, 280, 270, 270
}
int vel[] = {
  1, 3, 5, 7, 9, 9, 7, 5, 3, 1, 0
}

char buffer[128]; // buffer for printf

void dump_cur_joint_pos() {
  sprintf(buffer, "%3d %3d",
          cur_joint_pos[0],
          cur_joint_pos[1]
          );
  Serial.println(buffer);
}

void dump_cur_raw_pos() {
  sprintf(buffer, "%3f %3f",
          cur_raw_pos[0],
          cur_raw_pos[1]
          );
  Serial.println(buffer);
}

// reads current position estimate from odrive and stores in global array
void update_cur_pos() {
  odrv_rightarm_ser.write("r axis0.encoder.pos_estimate\n"); // right shoulder
  cur_raw_pos[0] = odrv_rightarm.readFloat();
  cur_joint_pos[0] = fmod(cur_raw_pos[0]*(360/GEAR_RATIO), 360);
  odrv_rightarm_ser.write("r axis1.encoder.pos_estimate\n"); // right elbow
  cur_raw_pos[1] = odrv_rightarm.readFloat();
  cur_joint_pos[1] = fmod(cur_raw_pos[1]*(360/GEAR_RATIO), 360);

  // verify everything is 0-360 for consistency
  for (int i = 0; i < NUM_JOINTS; i++) {
    cur_joint_pos[i] = cur_joint_pos[i] < 0 ? cur_joint_pos[i] + 360 : cur_joint_pos[i];
  }
}

// test if the current position is at the goal position
bool eval_at_goal(theta1, theta2) {
  // int r_sh = joint_angle_goal.right_shoulder;
  // int r_elb = joint_angle_goal.right_elbow;

  if (!(abs(cur_joint_pos[0] - theta1) <= DEADBAND)) {
    return false;
  }
  if (!(abs(cur_joint_pos[1] - theta2) <= DEADBAND)) {
    return false;
  }

  return true;
}

void setup() {
  Serial.begin(115200); // console

  // start serial connection to ODrives
  odrv_rightarm_ser.begin(115200);
}

void loop() {
  if (Serial.available() > 0) {
    Serial.read();
    for (int i = 0; i < sizeof(t1); i++) {
      // send velocity
      odrv_rightarm.SetVelocity(0, t1[i]);
      odrv_rightarm.SetVelocity(1, t2[i]);

      // wait for goal complete
      while (true) {
        update_cur_pos();
        if (eval_at_goal(t1[i], t2[i])) {
          break;
        }
      }
    }
  }
}