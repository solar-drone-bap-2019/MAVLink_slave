#include <mbed.h>
//#include "common/common.h"
//#include "protocol.h"
#include "mavlink.h"
#include "millis.h"

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"


DigitalOut myHigh(D10);
DigitalOut LED_BUILTIN(LED1);

// https://os.mbed.com/handbook/Serial
Serial pc(USBTX, USBRX);
Serial SerialMAV(PB_6, PB_7, 921600);

mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];

int type = MAV_TYPE_QUADROTOR; // Comments say this means airplane/fixed wing. I don't believe them

// Define the system type, in this case an airplane -> on-board controller
uint8_t system_type = MAV_TYPE_GENERIC;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

// Hardware definitions
uint8_t system_mode = MAV_MODE_TEST_ARMED; /// /* UNDEFINED mode. This solely depends on the autopilot - use with caution, intended for developers only. | */
uint32_t custom_mode = MAV_MODE_FLAG_SAFETY_ARMED; ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

// Mavlink variables
unsigned long previousMillisMAVLink = 0;     // will store last time MAVLink was transmitted and listened
unsigned long next_interval_MAVLink = 1000;  // next interval to count
const int num_hbs = 60;                      // # of heartbeats to wait before activating STREAMS from APM. 60 = one minute.
int num_hbs_past = num_hbs;




void mav_heartbeat_pack() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    /**
     * @brief Pack a heartbeat message
     * @param system_id ID of this system
     * @param component_id ID of this component (e.g. 200 for IMU)
     * @param msg The MAVLink message to compress the data into
     *
     * @param type Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
     * @param autopilot Autopilot type / class. defined in MAV_AUTOPILOT ENUM
     * @param base_mode System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
     * @param custom_mode A bitfield for use for autopilot-specific flags.
     * @param system_status System status flag, see MAV_STATE ENUM
     * @return length of the message in bytes (excluding serial stream start sign)
     */
    mavlink_msg_heartbeat_pack(255,0, &msg, type, autopilot_type, system_mode, custom_mode, system_state);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);
}

void mav_set_mode() {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    //Set flight mode 'Stabilize'
    mavlink_msg_set_mode_pack(0xFF, 0xBE, &msg, 1, 209, 0);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);
}

void mav_arm_pack(bool state) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    //Arm the drone
    //400 stands for MAV_CMD_COMPONENT_ARM_DISARM
    // 1 an 8'th argument is for ARM (0 for DISARM)
    if(state) {
        //ARM
        mavlink_msg_command_long_pack(0xFF, 0xBE, &msg, 1, 1, 400, 1,1.0,0,0,0,0,0,0);
    }else {
        //DISARM
        mavlink_msg_command_long_pack(0xFF, 0xBE, &msg, 1, 1, 400, 1,0.0,0,0,0,0,0,0);
    }
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialMAV.write(buf, len);
}

/* This function gets message from the APM and interpret for Mavlink common messages */
void comm_receive() {
    mavlink_message_t msg;
    mavlink_status_t status;

    //Indicates no data input
    LED_BUILTIN = false;

    //Checks if drone is connected
    while(SerialMAV.available()) {
        uint8_t c = SerialMAV.getc();
        //Indicates data receive frequency
        LED_BUILTIN = true;

        // Try to get a new message
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            //Indicates data flow
            pc.printf("+");
            // Handle message
            switch(msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
                {
                    // E.g. read GCS heartbeat and go into
                    // comm lost mode if timer times out
                    //Serial.println("MAVLINK_MSG_ID_HEARTBEAT");
                    mavlink_heartbeat_t hb;
                    mavlink_msg_heartbeat_decode(&msg, &hb);
                    pc.printf("State: "); pc.printf(hb.base_mode == 209 ? "Armed\n" : "Disarmed\n");
                    pc.printf("Mode: ");
                    switch(hb.custom_mode) {
                        case 0:
                            pc.printf("Stabilize\n");
                            break;
                        case 2:
                            pc.printf("AltHold\n");
                            break;
                        case 3:
                            pc.printf("Auto\n");
                            break;
                        case 5:
                            pc.printf("Loiter\n");
                            break;
                        case 7:
                            pc.printf("Circle\n");
                            break;
                        default:
                            pc.printf("Mode not known\n");
                            break;
                    }
                }
                    break;
                case MAVLINK_MSG_ID_SYS_STATUS:  // #1: SYS_STATUS
                {
                    /* Message decoding: PRIMITIVE
                     *    mavlink_msg_sys_status_decode(const mavlink_message_t* msg, mavlink_sys_status_t* sys_status)
                     */
                    mavlink_sys_status_t sys_status;
                    mavlink_msg_sys_status_decode(&msg, &sys_status);
                    //Serial.println("MAVLINK_MSG_ID_SYS_STATUS");
                    //Serial.println("Battery (V): ");
                    //Serial.println(sys_status.voltage_battery);
                }
                    break;
                case MAVLINK_MSG_ID_PARAM_VALUE:  // #22: PARAM_VALUE
                {
                    /* Message decoding: PRIMITIVE
                     *    mavlink_msg_param_value_decode(const mavlink_message_t* msg, mavlink_param_value_t* param_value)
                     */
                    //mavlink_message_t* msg;
                    mavlink_param_value_t param_value;
                    mavlink_msg_param_value_decode(&msg, &param_value);
                    //Serial.println("MAVLINK_MSG_ID_PARAM_VALUE");
                }
                    break;
                case MAVLINK_MSG_ID_RAW_IMU:  // #27: RAW_IMU
                {
                    /* Message decoding: PRIMITIVE
                     *    static inline void mavlink_msg_raw_imu_decode(const mavlink_message_t* msg, mavlink_raw_imu_t* raw_imu)
                     */
                    mavlink_raw_imu_t raw_imu;
                    mavlink_msg_raw_imu_decode(&msg, &raw_imu);
                    //Serial.println("MAVLINK_MSG_ID_RAW_IMU");
                }
                    break;
                case MAVLINK_MSG_ID_ATTITUDE:  // #30
                {
                    /* Message decoding: PRIMITIVE
                     *    mavlink_msg_attitude_decode(const mavlink_message_t* msg, mavlink_attitude_t* attitude)
                     */
                    mavlink_attitude_t attitude;
                    mavlink_msg_attitude_decode(&msg, &attitude);
                    //Serial.println("MAVLINK_MSG_ID_ATTITUDE");
                    //Serial.println("ROLL: ");
                    //Serial.println(attitude.roll);
                }
                    break;
                case MAVLINK_MSG_ID_SET_MODE: // #11
                {
                    mavlink_set_mode_t set_mode;
                    mavlink_msg_set_mode_decode(&msg, &set_mode);
                    /*
                    Serial.println("CUSTOM_MODE: ");
                    Serial.println(set_mode.custom_mode);
                    Serial.println("TARGET_SYSTEM: ");
                    Serial.println(set_mode.target_system);
                    Serial.println("BASE_MODE: ");
                    Serial.println(set_mode.base_mode);
                    */
                }
                    break;
                    //Not overriden channels
                case MAVLINK_MSG_ID_RC_CHANNELS_RAW: // #35
                {
                    /* 
                    *  RC (Radio controll) channels are the inputs and outputs for controlling all 
                    *  actions called from joystick / mission planner. E.g. arm, throttle, pitch.
                    */
                    mavlink_rc_channels_raw_t chs;
                    mavlink_msg_rc_channels_raw_decode(&msg, &chs);

                    /*pc.printf("Roll: ");  pc.printf(chs.chan1_raw);
                    Serial.println();
                    pc.printf("Pitch: ");  pc.printf(chs.chan2_raw + '\n');
                    Serial.println();
                    pc.printf("Throttle: ");  pc.printf(chs.chan3_raw + '\n');
                    Serial.println();*/
                }
                    break;
                    //Overriden channels for radio values
                case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: // #70
                {
                    mavlink_rc_channels_override_t ov_chs;
                    mavlink_msg_rc_channels_override_decode(&msg, &ov_chs);

                    /*pc.printf("Overr. Roll: ");  pc.printf(ov_chs.chan1_raw);
                    Serial.println();
                    pc.printf("Overr. Pitch: ");  pc.printf(ov_chs.chan2_raw + '\n');
                    Serial.println();
                    pc.printf("Overr. Throttle: ");  pc.printf(ov_chs.chan3_raw + '\n');
                    Serial.println();*/
                }
                    break;
            }
        }
    }
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
    const int  maxStreams = 1;
    const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_ALL};
    const uint16_t MAVRates[maxStreams] = {0x02};

    for (int i=0; i < maxStreams; i++) {
        /*
         * mavlink_msg_request_data_stream_pack(system_id, component_id, 
         *    &msg, 
         *    target_system, target_component, 
         *    MAV_DATA_STREAM_POSITION, 10000000, 1);
         *    
         * mavlink_msg_request_data_stream_pack(uint8_t system_id, uint8_t component_id, 
         *    mavlink_message_t* msg,
         *    uint8_t target_system, uint8_t target_component, uint8_t req_stream_id, 
         *    uint16_t req_message_rate, uint8_t start_stop)
         * 
         */

        mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        SerialMAV.write(buf, len);
    }
}

int main() {
    char buf[10];
    char msg[] = "Slave!";
    myHigh.write(1);
    millisStart();
    pc.printf("Hello everyone :D\n");

    while (1) {
        // We have to send the heartbeats to indicate side by side connection
        mav_heartbeat_pack();

        // Send the message with the standard UART send function
        // uart0_send might be named differently depending on
        // the individual microcontroller / library in use.
        unsigned long currentMillisMAVLink = millis();
        if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
            // Timing variables
            previousMillisMAVLink = currentMillisMAVLink;

            //SerialMAV.write(buf, len);

            //Mav_Request_Data();
            num_hbs_past++;
            if(num_hbs_past>=num_hbs) {
                // Request streams from APM
                Mav_Request_Data();
                num_hbs_past=0;
            }
        }

        // Check reception buffer
        comm_receive();
    }
}

#pragma clang diagnostic pop