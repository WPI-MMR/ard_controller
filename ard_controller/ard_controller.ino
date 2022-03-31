#include <ODriveArduino.h>
#include <HardwareSerial.h>
#include <Servo.h>
#include <math.h>

#define GEAR_RATIO 9
#define NUM_JOINTS 8
#define NUM_IMU_AXES 3
#define PREAMBLE_LENGTH 4
#define DATA_BYTE_LENGTH 2
#define DEADBAND 2

#define LEFT_FOOT_DOWN 51
#define LEFT_FOOT_UP 90
#define RIGHT_FOOT_DOWN 63
#define RIGHT_FOOT_UP 0

enum SerialReadState {
  INIT,
  READ_PREAMBLE,
  READ_DATA_REQUEST,
  READ_L_HIP,
  READ_L_KNEE,
  READ_R_HIP,
  READ_R_KNEE,
  READ_L_SHOULDER,
  READ_L_ELBOW,
  READ_R_SHOULDER,
  READ_R_ELBOW,
  READ_L_ANKLE,
  READ_R_ANKLE,
  READ_CHECKSUM
};

SerialReadState sr_state = INIT;

struct JointAngleStruct {
  int left_hip;
  int left_knee;
  int right_hip;
  int right_knee;
  int left_shoulder;
  int left_elbow;
  int right_shoulder;
  int right_elbow;
  int left_ankle;
  int right_ankle;
  int checksum;
  bool checksum_error;
  bool packet_available;
  bool data_request;
};

JointAngleStruct temporary_packet_data;
JointAngleStruct validated_packet_data;
JointAngleStruct joint_angle_goal;

// rename Serial1 to represent that it talks to the raspi
HardwareSerial& raspi_ser = Serial1;

// Serial2, 3, 4, and 5 are built in variables defined by the Teensy board definition
// these lines simply rename the serial objects to represent the odrive they're tied to
HardwareSerial& odrv_leftleg_ser = Serial5; // 1L + 2L
HardwareSerial& odrv_rightleg_ser = Serial3; // 1R + 2R
HardwareSerial& odrv_leftarm_ser = Serial4; // 3L + 4L
HardwareSerial& odrv_rightarm_ser = Serial2; // 3R + 4R

ODriveArduino odrv_leftleg(odrv_leftleg_ser);
ODriveArduino odrv_rightleg(odrv_rightleg_ser);
ODriveArduino odrv_leftarm(odrv_leftarm_ser);
ODriveArduino odrv_rightarm(odrv_rightarm_ser);

Servo left_ankle;
Servo right_ankle;

bool write_flag = false;

int cur_joint_pos[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};
float cur_raw_pos[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};

char buffer[128]; // buffer for printf

void dump_validated_packet_data() {
  sprintf(buffer, "%3u %3u %3u %3u %3u %3u %3u %3u %3u %3u %3u %3u %3u %3u",
          validated_packet_data.left_hip,
          validated_packet_data.left_knee,
          validated_packet_data.right_hip,
          validated_packet_data.right_knee,
          validated_packet_data.left_shoulder,
          validated_packet_data.left_elbow,
          validated_packet_data.right_shoulder,
          validated_packet_data.right_elbow,
          validated_packet_data.left_ankle,
          validated_packet_data.right_ankle,
          validated_packet_data.checksum,
          validated_packet_data.checksum_error,
          validated_packet_data.packet_available,
          validated_packet_data.data_request
          );
  Serial.println(buffer);
}

void dump_temporary_packet_data() {
  sprintf(buffer, "%3u %3u %3u %3u %3u %3u %3u %3u %3u %3u %3u %3u %3u %3u",
          temporary_packet_data.left_hip,
          temporary_packet_data.left_knee,
          temporary_packet_data.right_hip,
          temporary_packet_data.right_knee,
          temporary_packet_data.left_shoulder,
          temporary_packet_data.left_elbow,
          temporary_packet_data.right_shoulder,
          temporary_packet_data.right_elbow,
          validated_packet_data.left_ankle,
          validated_packet_data.right_ankle,
          temporary_packet_data.checksum,
          temporary_packet_data.checksum_error,
          temporary_packet_data.packet_available,
          temporary_packet_data.data_request
          );
  Serial.println(buffer);
}

void dump_cur_joint_pos() {
  sprintf(buffer, "%3d %3d %3d %3d %3d %3d %3d %3d",
          cur_joint_pos[0],
          cur_joint_pos[1],
          cur_joint_pos[2],
          cur_joint_pos[3],
          cur_joint_pos[4],
          cur_joint_pos[5],
          cur_joint_pos[6],
          cur_joint_pos[7]
          );
  Serial.println(buffer);
}

void dump_cur_raw_pos() {
  sprintf(buffer, "%3f %3f %3f %3f %3f %3f %3f %3f",
          cur_raw_pos[0],
          cur_raw_pos[1],
          cur_raw_pos[2],
          cur_raw_pos[3],
          cur_raw_pos[4],
          cur_raw_pos[5],
          cur_raw_pos[6],
          cur_raw_pos[7]
          );
  Serial.println(buffer);
}

void rx_processor() {
  static int preamble_counter = PREAMBLE_LENGTH;   // Number of ints in the packet preamble
  static int data_byte_counter = DATA_BYTE_LENGTH; // Number of ints per joint angle
  static int calculated_checksum = 0xFF; // We have to keep a running checksum of incoming data to verify validity
  int received_data; // The current int read from the RPi

  if (raspi_ser.available() > 0) {
    if (sr_state != INIT) {
      received_data = raspi_ser.read();
      Serial.println(received_data);
    }

    switch (sr_state)
    {
      case INIT:
        calculated_checksum = 0x00;
        preamble_counter = PREAMBLE_LENGTH;

        temporary_packet_data.left_hip = 0;
        temporary_packet_data.left_knee = 0;
        temporary_packet_data.right_hip = 0;
        temporary_packet_data.right_knee = 0;
        temporary_packet_data.left_shoulder = 0;
        temporary_packet_data.left_elbow = 0;
        temporary_packet_data.right_shoulder = 0;
        temporary_packet_data.right_elbow = 0;
        temporary_packet_data.left_ankle = 0;
        temporary_packet_data.right_ankle = 0;
        temporary_packet_data.checksum = 0;
        temporary_packet_data.checksum_error = false;
        temporary_packet_data.packet_available = false;
        temporary_packet_data.data_request = false;
        sr_state = READ_PREAMBLE;
        break;
      case READ_PREAMBLE:
        if (received_data == 0xFF) {
          preamble_counter--;
          if (preamble_counter == 0) {
            preamble_counter = PREAMBLE_LENGTH;
            sr_state = READ_DATA_REQUEST;
          }
        }
        else {
          preamble_counter = PREAMBLE_LENGTH;
        }
        break;
      case READ_DATA_REQUEST:
        if (received_data > 0) {
          temporary_packet_data.data_request = true;
          sr_state = READ_CHECKSUM;
        } else {
          temporary_packet_data.data_request = false;
          sr_state = READ_L_HIP;
        }
        calculated_checksum += received_data;
        break;
      case READ_L_HIP:
        temporary_packet_data.left_hip += received_data;
        calculated_checksum += received_data;
        data_byte_counter--;
        if (data_byte_counter == 0) {
          data_byte_counter = DATA_BYTE_LENGTH;
          sr_state = READ_L_KNEE;
        }
        break;
      case READ_L_KNEE:
        temporary_packet_data.left_knee += received_data;
        calculated_checksum += received_data;
        data_byte_counter--;
        if (data_byte_counter == 0) {
          data_byte_counter = DATA_BYTE_LENGTH;
          sr_state = READ_R_HIP;
        }
        break;
      case READ_R_HIP:
        temporary_packet_data.right_hip += received_data;
        calculated_checksum += received_data;
        data_byte_counter--;
        if (data_byte_counter == 0) {
          data_byte_counter = DATA_BYTE_LENGTH;
          sr_state = READ_R_KNEE;
        }
        break;
      case READ_R_KNEE:
        temporary_packet_data.right_knee += received_data;
        calculated_checksum += received_data;
        data_byte_counter--;
        if (data_byte_counter == 0) {
          data_byte_counter = DATA_BYTE_LENGTH;
          sr_state = READ_L_SHOULDER;
        }
        break;
      case READ_L_SHOULDER:
        temporary_packet_data.left_shoulder += received_data;
        calculated_checksum += received_data;
        data_byte_counter--;
        if (data_byte_counter == 0) {
          data_byte_counter = DATA_BYTE_LENGTH;
          sr_state = READ_L_ELBOW;
        }
        break;
      case READ_L_ELBOW:
        temporary_packet_data.left_elbow += received_data;
        calculated_checksum += received_data;
        data_byte_counter--;
        if (data_byte_counter == 0) {
          data_byte_counter = DATA_BYTE_LENGTH;
          sr_state = READ_R_SHOULDER;
        }
        break;
      case READ_R_SHOULDER:
        temporary_packet_data.right_shoulder += received_data;
        calculated_checksum += received_data;
        data_byte_counter--;
        if (data_byte_counter == 0) {
          data_byte_counter = DATA_BYTE_LENGTH;
          sr_state = READ_R_ELBOW;
        }
        break;
      case READ_R_ELBOW:
        temporary_packet_data.right_elbow += received_data;
        calculated_checksum += received_data;
        data_byte_counter--;
        if (data_byte_counter == 0) {
          data_byte_counter = DATA_BYTE_LENGTH;
          sr_state = READ_L_ANKLE;
        }
        break;
      case READ_L_ANKLE:
        temporary_packet_data.left_ankle = received_data;
        calculated_checksum += received_data;
        sr_state = READ_R_ANKLE;
        break;
      case READ_R_ANKLE:
        temporary_packet_data.right_ankle = received_data;
        calculated_checksum += received_data;
        sr_state = READ_CHECKSUM;
        break;
      case READ_CHECKSUM:
        temporary_packet_data.checksum = received_data;

        // calculated checksum must match received checksum; otherwise there's an error
        if (temporary_packet_data.checksum == 0xFF - (calculated_checksum % 256)) {
          // good packet
          calculated_checksum = 0x00;
          preamble_counter = PREAMBLE_LENGTH;

          temporary_packet_data.checksum_error = false;
          temporary_packet_data.packet_available = true;

          validated_packet_data.left_hip = temporary_packet_data.left_hip;
          validated_packet_data.left_knee = temporary_packet_data.left_knee;
          validated_packet_data.right_hip = temporary_packet_data.right_hip;
          validated_packet_data.right_knee = temporary_packet_data.right_knee;
          validated_packet_data.left_shoulder = temporary_packet_data.left_shoulder;
          validated_packet_data.left_elbow = temporary_packet_data.left_elbow;
          validated_packet_data.right_shoulder = temporary_packet_data.right_shoulder;
          validated_packet_data.right_elbow = temporary_packet_data.right_elbow;
          validated_packet_data.left_ankle = temporary_packet_data.left_ankle;
          validated_packet_data.right_ankle = temporary_packet_data.right_ankle;
          validated_packet_data.checksum = temporary_packet_data.checksum;
          validated_packet_data.checksum_error = temporary_packet_data.checksum_error;
          validated_packet_data.packet_available = temporary_packet_data.packet_available;
          validated_packet_data.data_request = temporary_packet_data.data_request;

          if (!validated_packet_data.data_request) {
            joint_angle_goal = temporary_packet_data;
          }

          sr_state = INIT;
        }
        else {
          temporary_packet_data.checksum_error = true;
          temporary_packet_data.packet_available = false;
          sr_state = INIT;
          Serial.print("Bad checksum. Calculated: ");
          Serial.println(0xFF - (calculated_checksum % 256));
        }
        break;
      default:
        sr_state = INIT;
        break;
      }
  }
}

// reads current position estimate from odrive and stores in global array
void update_cur_pos() {
  odrv_leftleg_ser.write("r axis0.encoder.pos_estimate\n"); // left hip
  cur_raw_pos[0] = odrv_leftleg.readFloat();
  cur_joint_pos[0] = fmod(cur_raw_pos[0]*(360/GEAR_RATIO), 360);
  odrv_leftleg_ser.write("r axis1.encoder.pos_estimate\n"); // left knee
  cur_raw_pos[1] = odrv_leftleg.readFloat();
  cur_joint_pos[1] = fmod(cur_raw_pos[1]*(360/GEAR_RATIO), 360);
  odrv_rightleg_ser.write("r axis0.encoder.pos_estimate\n"); // right hip
  cur_raw_pos[2] = odrv_rightleg.readFloat();
  cur_joint_pos[2] = fmod(cur_raw_pos[2]*(360/GEAR_RATIO), 360);
  odrv_rightleg_ser.write("r axis1.encoder.pos_estimate\n"); // right knee
  cur_raw_pos[3] = odrv_rightleg.readFloat();
  cur_joint_pos[3] = fmod(cur_raw_pos[3]*(360/GEAR_RATIO), 360);

  odrv_leftarm_ser.write("r axis0.encoder.pos_estimate\n"); // left shoulder
  cur_raw_pos[4] = odrv_leftarm.readFloat();
  cur_joint_pos[4] = fmod(cur_raw_pos[4]*(360/GEAR_RATIO), 360);
  odrv_leftarm_ser.write("r axis1.encoder.pos_estimate\n"); // left elbow
  cur_raw_pos[5] = odrv_leftarm.readFloat();
  cur_joint_pos[5] = fmod(cur_raw_pos[5]*(360/GEAR_RATIO), 360);
  odrv_rightarm_ser.write("r axis0.encoder.pos_estimate\n"); // right shoulder
  cur_raw_pos[6] = odrv_rightarm.readFloat();
  cur_joint_pos[6] = fmod(cur_raw_pos[6]*(360/GEAR_RATIO), 360);
  odrv_rightarm_ser.write("r axis1.encoder.pos_estimate\n"); // right elbow
  cur_raw_pos[7] = odrv_rightarm.readFloat();
  cur_joint_pos[7] = fmod(cur_raw_pos[7]*(360/GEAR_RATIO), 360);

  // verify everything is 0-360 for consistency
  for (int i = 0; i < NUM_JOINTS; i++) {
    cur_joint_pos[i] = cur_joint_pos[i] < 0 ? cur_joint_pos[i] + 360 : cur_joint_pos[i];
  }
}

// test if the current position is at the goal position
bool eval_at_goal() {
  int l_hip = joint_angle_goal.left_hip;
  int l_knee = joint_angle_goal.left_knee;
  int r_hip = joint_angle_goal.right_hip;
  int r_knee = joint_angle_goal.right_knee;
  int l_sh = joint_angle_goal.left_shoulder;
  int l_elb = joint_angle_goal.left_elbow;
  int r_sh = joint_angle_goal.right_shoulder;
  int r_elb = joint_angle_goal.right_elbow;

  if (!(abs(cur_joint_pos[0] - l_hip) <= DEADBAND)) {
    return false;
  }
  if (!(abs(cur_joint_pos[1] - l_knee) <= DEADBAND)) {
    return false;
  }
  if (!(abs(cur_joint_pos[2] - r_hip) <= DEADBAND)) {
    return false;
  }
  if (!(abs(cur_joint_pos[3] - r_knee) <= DEADBAND)) {
    return false;
  }
  if (!(abs(cur_joint_pos[4] - l_sh) <= DEADBAND)) {
    return false;
  }
  if (!(abs(cur_joint_pos[5] - l_elb) <= DEADBAND)) {
    return false;
  }
  if (!(abs(cur_joint_pos[6] - r_sh) <= DEADBAND)) {
    return false;
  }
  if (!(abs(cur_joint_pos[7] - r_elb) <= DEADBAND)) {
    return false;
  }

  return true;
}

void sensor_data_response() {
  int raw_sum = 0;
  int checksum;
  int angle;
  int b1, b2;

  int goal = eval_at_goal();
  // dump_cur_joint_pos();

  // send preamble
  for (int i = 0; i < PREAMBLE_LENGTH; i++) {
    raspi_ser.write((byte)255);
  }

  // send IMU data (sending 0's since IMU not installed)
  for (int i = 0; i < NUM_IMU_AXES; i++) {
    raspi_ser.write((byte)0);
    raspi_ser.write((byte)0);
  }

  // send current joint angle data
  for (int i = 0; i < NUM_JOINTS; i++) {
    angle = cur_joint_pos[i];
    raw_sum += angle;
    b1 = angle / 256 > 0 ? 255 : angle;
    b2 = angle / 256 > 0 ? angle % 255 : 0;

    raspi_ser.write((byte)b1);
    raspi_ser.write((byte)b2);
  }

  // send at_goal flag
  raw_sum += goal;
  raspi_ser.write((byte)goal);

  // send checksum
  checksum = 255 - raw_sum % 256;
  raspi_ser.write((byte)checksum);
}

int deg_dist(int initial, int final) {
  int diff = abs(final - initial) % 360;
  int result = diff > 180 ? 360 - diff : diff;
  int sign = (final-initial >= 0 && final-initial <=180) || (final-initial <= -180 && final-initial >= -360) ? 1 : -1;

  return result * sign;
}

void run_motors() {
  float left_hip_setpoint, left_knee_setpoint, right_hip_setpoint, right_knee_setpoint;
  float left_sh_setpoint, left_elb_setpoint, right_sh_setpoint, right_elb_setpoint;

  left_hip_setpoint = cur_raw_pos[0] + (deg_dist(cur_joint_pos[0], joint_angle_goal.left_hip) * (GEAR_RATIO/360.0));
  left_knee_setpoint = cur_raw_pos[1] + (deg_dist(cur_joint_pos[1], joint_angle_goal.left_knee) * (GEAR_RATIO/360.0));
  right_hip_setpoint = cur_raw_pos[2] + (deg_dist(cur_joint_pos[2], joint_angle_goal.right_hip) * (GEAR_RATIO/360.0));
  right_knee_setpoint = cur_raw_pos[3] + (deg_dist(cur_joint_pos[3], joint_angle_goal.right_knee) * (GEAR_RATIO/360.0));
  left_sh_setpoint = cur_raw_pos[4] + (deg_dist(cur_joint_pos[4], joint_angle_goal.left_shoulder) * (GEAR_RATIO/360.0));
  left_elb_setpoint = cur_raw_pos[5] + (deg_dist(cur_joint_pos[5], joint_angle_goal.left_elbow) * (GEAR_RATIO/360.0));
  right_sh_setpoint = cur_raw_pos[6] + (deg_dist(cur_joint_pos[6], joint_angle_goal.right_shoulder) * (GEAR_RATIO/360.0));
  right_elb_setpoint = cur_raw_pos[7] + (deg_dist(cur_joint_pos[7], joint_angle_goal.right_elbow) * (GEAR_RATIO/360.0));

  odrv_leftleg.SetPosition(0, left_hip_setpoint);
  odrv_leftleg.SetPosition(1, left_knee_setpoint);
  odrv_rightleg.SetPosition(0, right_hip_setpoint);
  odrv_rightleg.SetPosition(1, right_knee_setpoint);
  odrv_leftarm.SetPosition(0, left_sh_setpoint);
  odrv_leftarm.SetPosition(1, left_elb_setpoint);
  odrv_rightarm.SetPosition(0, right_sh_setpoint);
  odrv_rightarm.SetPosition(1, right_elb_setpoint);

  if (joint_angle_goal.left_ankle == 1 && joint_angle_goal.right_ankle == 1) {
    left_ankle.write(LEFT_FOOT_DOWN);
    right_ankle.write(RIGHT_FOOT_DOWN);
  } else {
    left_ankle.write(LEFT_FOOT_UP);
    right_ankle.write(RIGHT_FOOT_UP);    
  }
}

void setup() {
  Serial.begin(115200); // console
  raspi_ser.begin(115200); // raspi comms

  // start serial connection to ODrives
  odrv_leftleg_ser.begin(115200);
  odrv_rightleg_ser.begin(115200);
  odrv_leftarm_ser.begin(115200);
  odrv_rightarm_ser.begin(115200);

  // attach pins to servos
  left_ankle.attach(38);
  right_ankle.attach(39);

  // initialize goal to 0 so that if a data request comes in before a goal is given, no error occurs
  joint_angle_goal.left_hip = 0;
  joint_angle_goal.left_knee = 0;
  joint_angle_goal.right_hip = 0;
  joint_angle_goal.right_knee = 0;
  joint_angle_goal.left_shoulder = 0;
  joint_angle_goal.left_elbow = 0;
  joint_angle_goal.right_shoulder = 0;
  joint_angle_goal.right_elbow = 0;

  left_ankle.write(LEFT_FOOT_UP);
  right_ankle.write(RIGHT_FOOT_UP);
}

void loop() {
  rx_processor();
  if (validated_packet_data.packet_available) {
    // if (!validated_packet_data.data_request) {
    //   dump_validated_packet_data();
    // }
    dump_validated_packet_data();

    validated_packet_data.packet_available = false;
    update_cur_pos();

    if (validated_packet_data.data_request) {
      sensor_data_response();
    }
    else {
      // we received a new joint angle goal; get to it
      // dump_cur_joint_pos();
      run_motors();
    }
  }
}
