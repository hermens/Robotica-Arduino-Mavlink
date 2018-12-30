
#include "mavlink.h"

#define DistancePIN 7
#define AltitudePI 8


unsigned long previousMillisMAVLink = 0;    // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000; // next interval to count
const int num_hbs = 60;                     // # of heartbeats to wait before activating STREAMS from Pixhawk. 60 = one minute.
int num_hbs_pasados = num_hbs;


// MAVLINK Parameters (header)
int sysid = 1;                 ///< ID 20 for this airplane. 1 PX, 255 ground station
int compid = 158;              ///< The component sending the message
int type = MAV_TYPE_QUADROTOR; ///< This system is an airplane / fixed wing

// Define the system type, in this case an airplane -> on-board controller
// uint8_t system_type = MAV_TYPE_FIXED_WING;
uint8_t system_type = MAV_TYPE_GENERIC;
uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;

uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

void setup()
{
  // MAVLink interface start
  Serial.begin(57600);

  // Sensors setup
  pinMode(DistancePIN, INPUT);
  pinMode(AltitudePIN, INPUT);
}

double getFrontDistance()
{
  long duration = pulseIn(DistancePIN, HIGH);
  long distance = (duration/147)*2.54;
  Serial.print(distance);
  Serial.print("cm");
}


double getAltitude()
{
  long duration = pulseIn(AltitudePIN, HIGH);
  long distance = (duration/147)*2.54;
  Serial.print(distance);
  Serial.print("cm");
}

void sendHeartBeat()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(sysid, compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void takeOff()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_command_long_pack(sysid, compid, &msg, type, autopilot_type, MAV_CMD_NAV_TAKEOFF, 1, 0, 0, 0, 0, 0, 0, 1.0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void land()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(sysid, compid, &msg, type, autopilot_type, MAV_CMD_NAV_LAND, 1, 0, 0, 0, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

/**
 * @param state true for arm, false for disarm 
 */
void changeArmingState(boolean state)
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  //Arm the drone using MAV_CMD_COMPONENT_ARM_DISARM command
  // 1 an 8'th argument is for ARM (0 for DISARM)
  if (state)
  {
    // ARM
    mavlink_msg_command_long_pack(sysid, compid, &msg, type, autopilot_type, MAV_CMD_COMPONENT_ARM_DISARM, 1, 1.0, 0, 0, 0, 0, 0, 0 );
  }
  else
  {
    //DISARM
    // param2 to 21196 will disarm on flight
    mavlink_msg_command_long_pack(sysid, compid, &msg, type, autopilot_type, MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0, 0 );
  }

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial.write(buf, len);
}

void loop()
{

  unsigned long currentMillis = millis();

  sendHeartBeat();

  unsigned long currentMillisMAVLink = millis();

  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink)
  {

    // Guardamos la última vez que se cambió el modo
    previousMillisMAVLink = currentMillisMAVLink;


    num_hbs_pasados++;
    if (num_hbs_pasados >= 60) // after one minute
    {
      //Request streams from Pixhawk
      Mav_Request_Data();
      
      
      //num_hbs_pasados = 0;
    }

    if (num_hbs_pasados == 60)
    {
      changeArmingState(true);
    }

    if (num_hbs_pasados == 90)
    {
      takeOff();
    }

    if (num_hbs_pasados == 120)
    {
      land();
    }

    if (num_hbs_pasados == 130)
    {
      changeArmingState(false);
    }
  }

  // Check reception buffer
  comm_receive();
}

void Mav_Request_Data()
{
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // STREAMS that can be requested
  /*
   * Definitions are in common.h: enum MAV_DATA_STREAM
   *   
   * MAV_DATA_STREAM_ALL=0, // Enable all data streams
   * MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets.
   * MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS
   * MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW
   * MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT.
   * MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages.
   * MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot
   * MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot
   * MAV_DATA_STREAM_ENUM_END=13,
   * 
   * Data in PixHawk available in:
   *  - Battery, amperage and voltage (SYS_STATUS) in MAV_DATA_STREAM_EXTENDED_STATUS
   *  - Gyro info (IMU_SCALED) in MAV_DATA_STREAM_EXTRA1
   */

  // To be setup according to the needed information to be requested from the Pixhawk
  const int maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTRA1};
  const uint16_t MAVRates[maxStreams] = {0x02, 0x05};

  for (int i = 0; i < maxStreams; i++)
  {
    mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial.write(buf, len);
  }

}

void comm_receive()
{

  mavlink_message_t msg;
  mavlink_status_t status;

  // Echo for manual debugging
  // Serial.println("---Start---");

  while (Serial.available() > 0)
  {
    uint8_t c = Serial.read();

    // Try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {

      // Handle message
      switch (msg.msgid)
      {
      case MAVLINK_MSG_ID_HEARTBEAT: // #0: Heartbeat
      {
        Serial.println("Got a heartbeat");
      }
      break;

      case MAVLINK_MSG_ID_SYS_STATUS: // #1: SYS_STATUS
      {
        /* Message decoding: PRIMITIVE
             *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
             */
        //mavlink_message_t* msg;
        mavlink_sys_status_t sys_status;
        mavlink_msg_sys_status_decode(&msg, &sys_status);
      }
      break;

      case MAVLINK_MSG_ID_PARAM_VALUE: // #22: PARAM_VALUE
      {
        /* Message decoding: PRIMITIVE
             *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
             */
        //mavlink_message_t* msg;
        mavlink_param_value_t param_value;
        mavlink_msg_param_value_decode(&msg, &param_value);
      }
      break;

      case MAVLINK_MSG_ID_RAW_IMU: // #27: RAW_IMU
      {
        /* Message decoding: PRIMITIVE
             *    static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
             */
        mavlink_raw_imu_t raw_imu;
        mavlink_msg_raw_imu_decode(&msg, &raw_imu);
      }
      break;

      case MAVLINK_MSG_ID_ATTITUDE: // #30
      {
        /* Message decoding: PRIMITIVE
             *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
             */
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(&msg, &attitude);
        Serial.write("Roll: ");
        Serial.println(attitude.roll);

        Serial.write("Yaw: ");
        Serial.println(attitude.yaw);

        Serial.write("Pitch: ");
        Serial.println(attitude.pitch);

        
        delay(1000);
      }
      break;

      default:
        break;
      }
    }
  }
}
