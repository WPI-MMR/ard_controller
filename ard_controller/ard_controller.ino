#include <ODriveArduino.h>
#include <HardwareSerial.h>
#include <Servo.h>

#define GEAR_RATIO 9
#define NUM_JOINTS 8
#define NUM_IMU_AXES 3
#define PREAMBLE_LENGTH 4
#define DATA_BYTE_LENGTH 2

enum SerialReadState {
  INIT,
  READ_PREAMBLE,
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

HardwareSerial& odrv_leftleg_ser = Serial2;
HardwareSerial& odrv_rightleg_ser = Serial3;
HardwareSerial& odrv_leftarm_ser = Serial4;
HardwareSerial& odrv_rightarm_ser = Serial5;

ODriveArduino odrv_leftleg(odrv_leftleg_ser);
ODriveArduino odrv_rightleg(odrv_rightleg_ser);
ODriveArduino odrv_leftarm(odrv_leftarm_ser);
ODriveArduino odrv_rightarm(odrv_rightarm_ser);

bool write_flag = false;

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

void rx_processor() {
  static int preamble_counter = PREAMBLE_LENGTH;   // Number of ints in the packet preamble
  static int data_byte_counter = DATA_BYTE_LENGTH; // Number of ints per joint angle
  static int calculated_checksum = 0xFF; // We have to keep a running checksum of incoming data to verify validity
  int received_data; // The current int read from the RPi

  if (Serial1.available() > 0) {
    if (sr_state != INIT) {
      received_data = Serial1.read();
      // Serial.println(received_data);
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
            sr_state = READ_L_HIP;
          }
        }
        else {
          preamble_counter = PREAMBLE_LENGTH;
        }
        break;
      case READ_L_HIP:
        // this is different than other cases as it needs to catch a request for sensor data
        if (data_byte_counter == 1 && temporary_packet_data.left_hip == received_data) {
          temporary_packet_data.data_request = true;
          temporary_packet_data.left_hip += received_data;
          calculated_checksum += received_data;
          data_byte_counter = DATA_BYTE_LENGTH;
          sr_state = READ_CHECKSUM;
        }
        else {
          temporary_packet_data.left_hip += received_data;
          calculated_checksum += received_data;
          data_byte_counter--;
          if (data_byte_counter == 0) {
            data_byte_counter = DATA_BYTE_LENGTH;
            sr_state = READ_L_KNEE;
          }
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

          sr_state = INIT;
        }
        else {
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

void sensor_data_response() {
  int raw_sum = 0;
  int checksum;
  long angle, goal;
  int b1, b2;

  // send preamble
  for (int i = 0; i < PREAMBLE_LENGTH; i++) {
    Serial1.write((byte)255);
  }

  // send imu and joint angle data
  for (int i = 0; i < NUM_JOINTS + NUM_IMU_AXES; i++) {
    angle = random(360);
    raw_sum += angle;
    b1 = angle / 256 > 0 ? 255 : angle;
    b2 = angle / 256 > 0 ? angle % 255 : 0;

    Serial1.write((byte)b1);
    Serial1.write((byte)b2);
  }

  // send at_goal flag
  goal = random(2);
  raw_sum += goal;
  Serial1.write((byte)goal);

  // send checksum
  checksum = 255 - raw_sum % 256;
  Serial1.write((byte)checksum);
}

void setup() {
  Serial.begin(115200); // console
  Serial1.begin(115200); // raspi comms

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
}

void loop() {
  rx_processor();
  if (validated_packet_data.packet_available) {
    dump_validated_packet_data();
    validated_packet_data.packet_available = false;

    if (validated_packet_data.data_request) {
      sensor_data_response();
    }
    else {
      // we received a new joint angle goal; get to it
      odrv_leftleg.SetPosition(0, validated_packet_data.left_hip * GEAR_RATIO);
      odrv_rightleg.SetPosition(0, validated_packet_data.right_hip * GEAR_RATIO);
      odrv_leftarm.SetPosition(0, validated_packet_data.left_shoulder * GEAR_RATIO);
      odrv_rightarm.SetPosition(0, validated_packet_data.right_shoulder * GEAR_RATIO);
      odrv_leftleg.SetPosition(1, validated_packet_data.left_knee * GEAR_RATIO);
      odrv_rightleg.SetPosition(1, validated_packet_data.right_knee * GEAR_RATIO);
      odrv_leftarm.SetPosition(1, validated_packet_data.left_elbow * GEAR_RATIO);
      odrv_rightarm.SetPosition(1, validated_packet_data.right_elbow * GEAR_RATIO);
    }
  }
}
