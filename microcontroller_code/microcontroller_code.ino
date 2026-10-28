#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>


#define BAUD_RATE          57600
#define SYNC_BYTE          0xAA

#define OPCODE_RX_VELOCITY 0x00
#define OPCODE_TX_IMU      0x00
#define OPCODE_TX_GPS      0x01


float test1 = 0.1;
float test2 = 0.2;
float test3 = 1;


struct __attribute__((packed)) VelocityPayload {
    float linear_x;
    float angular_z;
};



struct __attribute__((packed)) ImuPayload {
    float  orientation[4];                    // w, x, y, z
    double orientation_covariance[9];         // row major 3x3 matrix
    float  angular_velocity[3];               // x, y, z
    double angular_velocity_covariance[9];    // row major 3x3 matrix
    float  linear_acceleration[3];            // x, y, z
    double linear_acceleration_covariance[9]; // row major 3x3 matrix
};


struct __attribute__((packed)) GpsPayload {
    double longitude;
    double latitude;
    double altitude;
    double position_covariance[9];            // row-major 3x3
};




// Just a function to compute crc8 (8 bit error correction) that can be found literally anywhere online
uint8_t compute_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            }
            else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Helper to compute CRC over (opcode + payload) without using a temporary variable length buffer
uint8_t compute_packet_crc8(uint8_t opcode, const uint8_t *payload, size_t payload_len) {
    uint8_t crc = compute_crc8(&opcode, 1);

    for (size_t i = 0; i < payload_len; i++) {
        crc ^= payload[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            }
            else {
                crc <<= 1;
            }
        }
    }
    return crc;
}




// Placeholder Functions
bool read_imu(ImuPayload &imu_data) {

    // Test Data

    imu_data.orientation[0] = 1.0;
    imu_data.orientation[1] = 0.0;
    imu_data.orientation[2] = 0.0;
    imu_data.orientation[3] = 0.0;

    imu_data.orientation_covariance[0] = 0.4;
    imu_data.orientation_covariance[1] = 0.0;
    imu_data.orientation_covariance[2] = 0.0;
    imu_data.orientation_covariance[3] = 0.0;
    imu_data.orientation_covariance[4] = 0.1;
    imu_data.orientation_covariance[5] = 0.0;
    imu_data.orientation_covariance[6] = 0.0;
    imu_data.orientation_covariance[7] = 0.0;
    imu_data.orientation_covariance[8] = 0.1;

    imu_data.angular_velocity[0] = 0.3;
    imu_data.angular_velocity[1] = 0.2;
    imu_data.angular_velocity[2] = 0.1;

    imu_data.angular_velocity_covariance[0] = 0.1;
    imu_data.angular_velocity_covariance[1] = 0.0;
    imu_data.angular_velocity_covariance[2] = 0.0;
    imu_data.angular_velocity_covariance[3] = 0.0;
    imu_data.angular_velocity_covariance[4] = 0.1;
    imu_data.angular_velocity_covariance[5] = 0.0;
    imu_data.angular_velocity_covariance[6] = 0.0;
    imu_data.angular_velocity_covariance[7] = 0.0;
    imu_data.angular_velocity_covariance[8] = 0.1;

    imu_data.linear_acceleration[0] = test1;
    imu_data.linear_acceleration[1] = test2;
    imu_data.linear_acceleration[2] = test3;

    // test3++;

    imu_data.linear_acceleration_covariance[0] = 0.1;
    imu_data.linear_acceleration_covariance[1] = 0.0;
    imu_data.linear_acceleration_covariance[2] = 0.0;
    imu_data.linear_acceleration_covariance[3] = 0.0;
    imu_data.linear_acceleration_covariance[4] = 0.1;
    imu_data.linear_acceleration_covariance[5] = 0.0;
    imu_data.linear_acceleration_covariance[6] = 0.0;
    imu_data.linear_acceleration_covariance[7] = 0.0;
    imu_data.linear_acceleration_covariance[8] = 0.1;

    return true;
}

bool read_gps(GpsPayload &gps_data) {

    gps_data.longitude = -80.4139;
    gps_data.latitude = 37.2296;
    gps_data.altitude = 0.6;

    gps_data.position_covariance[0] = 0.8;
    gps_data.position_covariance[1] = 0.0;
    gps_data.position_covariance[2] = 0.0;
    gps_data.position_covariance[3] = 0.0;
    gps_data.position_covariance[4] = 0.5;
    gps_data.position_covariance[5] = 0.0;
    gps_data.position_covariance[6] = 0.0;
    gps_data.position_covariance[7] = 0.0;
    gps_data.position_covariance[8] = 0.5;

    return true;
}


void set_robot_desired_velocity(float linear_x, float angular_z) {
    test1 = linear_x;
    test2 = angular_z;
}




void send_packet(uint8_t opcode, const void *payload, size_t payload_size) {
    const uint8_t *payload_bytes = static_cast<const uint8_t *>(payload);
    uint8_t crc = compute_packet_crc8(opcode, payload_bytes, payload_size);

    Serial.write(SYNC_BYTE);
    Serial.write(opcode);
    Serial.write(payload_bytes, payload_size);
    Serial.write(crc);
}

void send_imu_data() {
    ImuPayload imu_data = {};
    if (read_imu(imu_data)) {
        send_packet(OPCODE_TX_IMU, &imu_data, sizeof(ImuPayload));
    }
}

void send_gps_data() {
    GpsPayload gps_data = {};
    if (read_gps(gps_data)) {
        send_packet(OPCODE_TX_GPS, &gps_data, sizeof(GpsPayload));
    }
}





enum RxState {
    WAIT_SYNC,
    WAIT_OPCODE,
    WAIT_PAYLOAD,
    WAIT_CRC
};

RxState rx_state = WAIT_SYNC;
uint8_t rx_opcode = 0;
uint8_t rx_buffer[sizeof(VelocityPayload)];
size_t  rx_index = 0;

void read_incoming_packets() {
    while (Serial.available() > 0) {
        uint8_t byte_in = Serial.read();

        switch (rx_state) {
            case WAIT_SYNC:
                if (byte_in == SYNC_BYTE) {
                    rx_state = WAIT_OPCODE;
                }
                break;

            case WAIT_OPCODE:
                rx_opcode = byte_in;
                if (rx_opcode == OPCODE_RX_VELOCITY) {
                    rx_index = 0;
                    rx_state = WAIT_PAYLOAD;
                }
                else {
                    // Unknown opcode, so we should go back to hunting for sync byte
                    rx_state = WAIT_SYNC;
                }
                break;

            case WAIT_PAYLOAD:
                rx_buffer[rx_index++] = byte_in;
                if (rx_index >= sizeof(VelocityPayload)) {
                    rx_state = WAIT_CRC;
                }
                break;

            case WAIT_CRC: {
                uint8_t expected_crc = compute_packet_crc8(rx_opcode, rx_buffer, sizeof(VelocityPayload));

                if (byte_in == expected_crc) {
                    VelocityPayload cmd;
                    memcpy(&cmd, rx_buffer, sizeof(VelocityPayload));
                    set_robot_desired_velocity(cmd.linear_x, cmd.angular_z);
                }

                rx_state = WAIT_SYNC;
                break;
            }
        }
    }
}







unsigned long last_imu_time_ms = 0;
unsigned long last_gps_time_ms = 0;

const unsigned long IMU_INTERVAL_MS = 10;   // 100 Hz
const unsigned long GPS_INTERVAL_MS = 100;  // 10 Hz

void setup() {
    Serial.begin(BAUD_RATE);
}

void loop() {
    // Continuously parse incoming velocity commands whenever they are available
    read_incoming_packets();

    unsigned long time_now = millis();

    // Transmit IMU data at 100 Hz
    if (time_now - last_imu_time_ms >= IMU_INTERVAL_MS) {
        last_imu_time_ms = time_now;
        send_imu_data();
    }

    // Transmit GPS data at 10 Hz
    if (time_now - last_gps_time_ms >= GPS_INTERVAL_MS) {
        last_gps_time_ms = time_now;
        send_gps_data();
    }

    // Later on add a small controller that allows us to move the robot to get to the velocity command from the set_robot_desired_velocity function
}





