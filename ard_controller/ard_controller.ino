#include <ODriveArduino.h>
#include <HardwareSerial.h>
#include <Servo.h>
#include <math.h>

#define GEAR_RATIO 9
#define NUM_JOINTS 8
#define NUM_IMU_AXES 3
#define PREAMBLE_LENGTH 4
#define DATA_BYTE_LENGTH 2
#define DEADBAND 3

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
HardwareSerial& odrv_rightleg_ser = Serial3;
HardwareSerial& odrv_leftleg_ser = Serial2;
HardwareSerial& odrv_rightarm_ser = Serial5;
HardwareSerial& odrv_leftarm_ser = Serial4;

ODriveArduino odrv_leftleg(odrv_leftleg_ser);
ODriveArduino odrv_rightleg(odrv_rightleg_ser);
ODriveArduino odrv_leftarm(odrv_leftarm_ser);
ODriveArduino odrv_rightarm(odrv_rightarm_ser);

bool write_flag = false;

int cur_joint_pos[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};

char buffer[128]; // buffer for printf

void dump_validated_packet_data() {
  // 353   3   4   4   5   1   5 358  34   0   0   0
  sprintf(buffer, "%3u %3u %3u %3u %3u %3u %3u %3u %3u %3u %3u %3u",
          validated_packet_data.left_hip,
          validated_packet_data.left_knee,
          validated_packet_data.right_hip,
          validated_packet_data.right_knee,
          validated_packet_data.left_shoulder,
          validated_packet_data.left_elbow,
          validated_packet_data.right_shoulder,
          validated_packet_data.right_elbow,
          validated_packet_data.checksum,
          validated_packet_data.checksum_error,
          validated_packet_data.packet_available,
          validated_packet_data.data_request
          );
  Serial.println(buffer);
}

void dump_temporary_packet_data() {
  sprintf(buffer, "%3u %3u %3u %3u %3u %3u %3u %3u %3u %3u %3u %3u",
          temporary_packet_data.left_hip,
          temporary_packet_data.left_knee,
          temporary_packet_data.right_hip,
          temporary_packet_data.right_knee,
          temporary_packet_data.left_shoulder,
          temporary_packet_data.left_elbow,
          temporary_packet_data.right_shoulder,
          temporary_packet_data.right_elbow,
          temporary_packet_data.checksum,
          temporary_packet_data.checksum_error,
          temporary_packet_data.packet_available,
          temporary_packet_data.data_request
          );
  Serial.println(buffer);
}

void dump_cur_joint_pos() {
  sprintf(buffer, "%3u %3u %3u %3u %3u %3u %3u %3u",
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

void rx_processor() {
  static int preamble_counter = PREAMBLE_LENGTH;   // Number of ints in the packet preamble
  static int data_byte_counter = DATA_BYTE_LENGTH; // Number of ints per joint angle
  static int calculated_checksum = 0xFF; // We have to keep a running checksum of incoming data to verify validity
  int received_data; // The current int read from the RPi

  if (raspi_ser.available() > 0) {
    if (sr_state != INIT) {
      received_data = raspi_ser.read();
      //Serial.println(received_data);
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
          //Serial.println("Data request");
          temporary_packet_data.data_request = true;
          sr_state = READ_CHECKSUM;
        } else {
          //Serial.println("Joint angles");
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
          sr_state = READ_CHECKSUM;
        }
        break;
      case READ_CHECKSUM:
        temporary_packet_data.checksum = received_data;

        // calculated checksum must match received checksum; otherwise there's an error
        if (temporary_packet_data.checksum == 0xFF - (calculated_checksum % 256)) {
          // good packet
          //Serial.println("Good checksum");
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
          //Serial.println("Bad checksum");
          temporary_packet_data.checksum_error = true;
          temporary_packet_data.packet_available = false;
          sr_state = INIT;
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
  cur_joint_pos[0] = constrain(fmod(odrv_leftleg.readFloat()*(360/GEAR_RATIO), 360), 0, 360);
  odrv_leftleg_ser.write("r axis1.encoder.pos_estimate\n"); // left knee
  cur_joint_pos[1] = constrain(fmod(odrv_leftleg.readFloat()*(360/GEAR_RATIO), 360), 0, 360);
  odrv_rightleg_ser.write("r axis0.encoder.pos_estimate\n"); // right hip
  cur_joint_pos[2] = constrain(fmod(odrv_rightleg.readFloat()*(360/GEAR_RATIO), 360), 0, 360);
  odrv_rightleg_ser.write("r axis1.encoder.pos_estimate\n"); // right knee
  cur_joint_pos[3] = constrain(fmod(odrv_rightleg.readFloat()*(360/GEAR_RATIO), 360), 0, 360);

  odrv_leftarm_ser.write("r axis0.encoder.pos_estimate\n"); // left shoulder
  cur_joint_pos[4] = constrain(fmod(odrv_leftarm.readFloat()*(360/GEAR_RATIO), 360), 0, 360);
  odrv_leftarm_ser.write("r axis1.encoder.pos_estimate\n"); // left elbow
  cur_joint_pos[5] = constrain(fmod(odrv_leftarm.readFloat()*(360/GEAR_RATIO), 360), 0, 360);
  odrv_rightarm_ser.write("r axis0.encoder.pos_estimate\n"); // right shoulder
  cur_joint_pos[6] = constrain(fmod(odrv_rightarm.readFloat()*(360/GEAR_RATIO), 360), 0, 360);
  odrv_rightarm_ser.write("r axis1.encoder.pos_estimate\n"); // right elbow
  cur_joint_pos[7] = constrain(fmod(odrv_rightarm.readFloat()*(360/GEAR_RATIO), 360), 0, 360);
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

void setup() {
  delay(10000);
  Serial.begin(115200); // console
  Serial.print("Delay over");
  raspi_ser.begin(115200); // raspi comms

  // start serial connection to ODrives
  odrv_leftleg_ser.begin(115200);
  odrv_rightleg_ser.begin(115200);
  odrv_leftarm_ser.begin(115200);
  odrv_rightarm_ser.begin(115200);

  // set all ODrive axes to closed loop control
  int requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
  odrv_leftleg.run_state(0, requested_state, false); // hips
  odrv_rightleg.run_state(0, requested_state, false);
  odrv_leftarm.run_state(0, requested_state, false); // shoulders
  odrv_rightarm.run_state(0, requested_state, false);
  odrv_leftleg.run_state(1, requested_state, false); // knees
  odrv_rightleg.run_state(1, requested_state, false);
  odrv_leftarm.run_state(1, requested_state, false); // elbows
  odrv_rightarm.run_state(1, requested_state, false);

  // initialize goal to 0 so that if a data request comes in before a goal is given, no error occurs
  joint_angle_goal.left_hip = 0;
  joint_angle_goal.left_knee = 0;
  joint_angle_goal.right_hip = 0;
  joint_angle_goal.right_knee = 0;
  joint_angle_goal.left_shoulder = 0;
  joint_angle_goal.left_elbow = 0;
  joint_angle_goal.right_shoulder = 0;
  joint_angle_goal.right_elbow = 0;
}

int convert_to_half_range(int angle, int home_offset){
  if (angle > 180){
    angle = angle - 360;
  }
  angle = angle + home_offset;
  return constrain(angle, -90, 90);
}

void loop() {

 
  rx_processor();
  if (validated_packet_data.packet_available) {
    // dump_validated_packet_data();
    validated_packet_data.packet_available = false;

    if (validated_packet_data.data_request) {
      //Serial.println("Requesting data");
      update_cur_pos();
      sensor_data_response();
    }
    else {
      //Home offset value in terms of dump validated data format
      ///353   3   4   4   5   1   5 358  34   0   0   0
      dump_validated_packet_data();
      // we received a new joint angle goal; get to it
      odrv_leftleg.SetPosition(0, (-convert_to_half_range(validated_packet_data.left_hip, -7) * GEAR_RATIO) / 360.0);
      odrv_rightleg.SetPosition(0, (convert_to_half_range(validated_packet_data.right_hip, 5) * GEAR_RATIO) / 360.0);
      odrv_leftarm.SetPosition(0, (-convert_to_half_range(validated_packet_data.left_shoulder, -2) * GEAR_RATIO) / 360.0);
      odrv_rightarm.SetPosition(0, (convert_to_half_range(validated_packet_data.right_shoulder, 5) * GEAR_RATIO) / 360.0);
      odrv_leftleg.SetPosition(1, (-convert_to_half_range(validated_packet_data.left_knee, -5) * GEAR_RATIO) / 360.0);
      odrv_rightleg.SetPosition(1, (convert_to_half_range(validated_packet_data.right_knee, 4) * GEAR_RATIO) / 360.0);
      odrv_leftarm.SetPosition(1, (-convert_to_half_range(validated_packet_data.left_elbow, -7) * GEAR_RATIO) / 360.0);
      odrv_rightarm.SetPosition(1, (convert_to_half_range(validated_packet_data.right_elbow, 1) * GEAR_RATIO) / 360.0);

//      odrv_leftleg.SetPosition(0, (convert_to_half_range(validated_packet_data.left_hip, -7) * GEAR_RATIO) / 360.0);
//      odrv_rightleg.SetPosition(0, (-convert_to_half_range(validated_packet_data.right_hip, 4) * GEAR_RATIO) / 360.0);
//      odrv_leftarm.SetPosition(0, (convert_to_half_range(validated_packet_data.left_shoulder, 5) * GEAR_RATIO) / 360.0);
//      odrv_rightarm.SetPosition(0, (-convert_to_half_range(validated_packet_data.right_shoulder, 5) * GEAR_RATIO) / 360.0);
//      odrv_leftleg.SetPosition(1, (convert_to_half_range(validated_packet_data.left_knee, 3) * GEAR_RATIO) / 360.0);
//      odrv_rightleg.SetPosition(1, (-convert_to_half_range(validated_packet_data.right_knee, 4) * GEAR_RATIO) / 360.0);
//      odrv_leftarm.SetPosition(1, (convert_to_half_range(validated_packet_data.left_elbow, 1) * GEAR_RATIO) / 360.0);
//      odrv_rightarm.SetPosition(1, (-convert_to_half_range(validated_packet_data.right_elbow, -2) * GEAR_RATIO) / 360.0);
    }
  }
}
