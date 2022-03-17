#include <HardwareSerial.h>
#include <math.h>

#define GEAR_RATIO 9
#define NUM_JOINTS 8
#define NUM_IMU_AXES 3
#define PREAMBLE_LENGTH 4
#define DATA_BYTE_LENGTH 2
#define DEADBAND 2

#define NO_ACK 0
#define SUCCESS_ACK 1
#define FAILURE_ACK 2

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

bool write_flag = false;
bool send_ack = false;
bool ack_error = false;

long timestamp = millis();

int cur_joint_pos[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};
float cur_raw_pos[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};

char buffer[128]; // buffer for printf

void dump_validated_packet_data() {
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
          sr_state = READ_CHECKSUM;
        }
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
          validated_packet_data.checksum = temporary_packet_data.checksum;
          validated_packet_data.checksum_error = temporary_packet_data.checksum_error;
          validated_packet_data.packet_available = temporary_packet_data.packet_available;
          validated_packet_data.data_request = temporary_packet_data.data_request;

          if (!validated_packet_data.data_request) {
            joint_angle_goal = validated_packet_data;
          }

          sr_state = INIT;

          // send success ack
          send_ack = true;
          ack_error = false;
        }
        else {
          temporary_packet_data.checksum_error = true;
          temporary_packet_data.packet_available = false;
          sr_state = INIT;

          // send failure ack
          send_ack = true;
          ack_error = true;
        }
        break;
      default:
        sr_state = INIT;
        break;
      }
  }
}

void update_cur_pos() {
  int l_hip = joint_angle_goal.left_hip;
  int l_knee = joint_angle_goal.left_knee;
  int r_hip = joint_angle_goal.right_hip;
  int r_knee = joint_angle_goal.right_knee;
  int l_sh = joint_angle_goal.left_shoulder;
  int l_elb = joint_angle_goal.left_elbow;
  int r_sh = joint_angle_goal.right_shoulder;
  int r_elb = joint_angle_goal.right_elbow;

  if (cur_joint_pos[0] < l_hip) {cur_joint_pos[0] += 1;}
  else if (cur_joint_pos[0] > l_hip) {cur_joint_pos[0] -= 1;}
  if (cur_joint_pos[1] < l_knee) {cur_joint_pos[1] += 1;}
  else if (cur_joint_pos[1] > l_knee) {cur_joint_pos[1] -= 1;}
  if (cur_joint_pos[2] < r_hip) {cur_joint_pos[2] += 1;}
  else if (cur_joint_pos[2] > r_hip) {cur_joint_pos[2] -= 1;}
  if (cur_joint_pos[3] < r_knee) {cur_joint_pos[3] += 1;}
  else if (cur_joint_pos[3] > r_knee) {cur_joint_pos[3] -= 1;}
  if (cur_joint_pos[4] < l_sh) {cur_joint_pos[4] += 1;}
  else if (cur_joint_pos[4] > l_sh) {cur_joint_pos[4] -= 1;}
  if (cur_joint_pos[5] < l_elb) {cur_joint_pos[5] += 1;}
  else if (cur_joint_pos[5] > l_elb) {cur_joint_pos[5] -= 1;}
  if (cur_joint_pos[6] < r_sh) {cur_joint_pos[6] += 1;}
  else if (cur_joint_pos[6] > r_sh) {cur_joint_pos[6] -= 1;}
  if (cur_joint_pos[7] < r_elb) {cur_joint_pos[7] += 1;}
  else if (cur_joint_pos[7] > r_elb) {cur_joint_pos[7] -= 1;}
}

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

void data_response() {
  Serial.println("=========SENDING FOLLOWING RESPONSE==========");
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
    Serial.println(angle);
  }

  // send at_goal flag
  raw_sum += goal;
  raspi_ser.write((byte)goal);
  Serial.println(goal);

  //send 'setpoint ack' byte
  if (send_ack) {
    if (ack_error) {
      raspi_ser.write((byte)FAILURE_ACK);
      raw_sum += FAILURE_ACK;
      Serial.println(FAILURE_ACK);
    }
    else {
      raspi_ser.write((byte)SUCCESS_ACK);
      raw_sum += SUCCESS_ACK;
      Serial.println(SUCCESS_ACK);
    }
  }
  else {
    raspi_ser.write((byte)NO_ACK);
    raw_sum += NO_ACK;
    Serial.println(NO_ACK);
  }

  // send checksum
  checksum = 255 - raw_sum % 256;
  raspi_ser.write((byte)checksum);

  Serial.println("=========END RESPONSE==========");
}

void setup() {
  Serial.begin(115200); // console
  raspi_ser.begin(115200); // raspi comms

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

void loop() {
  rx_processor();
  if (validated_packet_data.packet_available) {
    // if (!validated_packet_data.data_request) {
    //   dump_validated_packet_data();
    // }
    dump_validated_packet_data();

    validated_packet_data.packet_available = false;
    data_response();

    if (send_ack) {
      send_ack = false;
    }
  }
  else if (temporary_packet_data.checksum_error) {
    data_response();
  }

  if (millis() - timestamp > 3) {
    update_cur_pos();
    timestamp = millis();
  }
}
