#!/usr/bin/python3

# ---------------------------------------------------------------
# Protocol Definition:
#
#   Baud Rate: 57600
#
#   Communication to the microcontroller happens every time the timer_callback is called.
#   The general message format is as follows:
#       Sync Byte (0xAA) followed by Opcode (8 bit) followed by Arguments (variable number of bits) followed by Checksum (8 bit)
#
#
#   Available Opcodes For Sending Data From Jetson to Microcontroller:
#       Send Velocity Command (opcode = 0x00)
#           Arguments: Desired linear x velocity (32-bit float, m/s),
#                      Desired angular z velocity (32-bit float, rad/s)
#
#           Note: Measured with respect to the robot's frame of reference:
#                 +x is forward, +z is counter clockwise yaw (turning left).
#
#
#   Available Opcodes For Sending Data from Microcontroller to Jetson:
#       Update Current IMU data (opcode = 0x00)
#           Arguments: orientation wxyz (32 * 4), orientation covariance matrix in row major order (64 * 9),
#                      angular_velocity (32 * 3), angular velocity covariance matrix in row major order (64 * 9)
#                      linear_acceleration (32 * 3), linear acceleration covariance matrix in row major order (64 * 9)
#
#       Update Current GPS data (opcode = 0x01)
#           Arguments: longitude (64 bits), latitude (64 bits), altitude (64 bits), position covariance matrix in row major order (64 * 9 bits)
#
#
#   TODO: ADD THE MESSAGE LENGTH IN BYTES RIGHT AFTER THE OPCODE SO THAT WE KNOW WHEN THE MESSAGE SHOULD TERMINATE
#   TODO: If we can get velocity data and covariance from the GPS that would be great! (we would need to also modify ekf.yaml)
# ---------------------------------------------------------------


# TODO: WE NEED TO MAKE SURE THAT THE ROBOT IS OUTPUTTING MESSAGES IN THE RIGHT FRAME OF REFERENCE (AKA LINKS IN ROS2)
# We would like the conventions to be the following out of this node:
#   linear acceleration is in FLU (aka forward=x, left=y, up=z)
#   angular velocity is in FLU (aka forward=x, left=y, up=z)
#   orientation is in FLU relative to ENU (aka east=x, north=y, up=z)
#   GPS position is in WGS84 (aka longitude/ latitude/ altitude)
#   position covariance is in ENU (aka east=x, north=y, up=z)



import math
from datetime import date
from pygeomag import GeoMag
from scipy.spatial.transform import Rotation as R


import struct
import serial
from serial.tools import list_ports

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu



MICROCONTROLLER_VID = 0x2E8A
MICROCONTROLLER_PID = 0x000A
BAUD_RATE = 57600
REFRESH_RATE = 100  # HERTZ

# Opcodes
SYNC_BYTE          = 0xAA
OPCODE_TX_VELOCITY = 0x00
OPCODE_RX_IMU      = 0x00
OPCODE_RX_GPS      = 0x01


# TX Velocity Payload Format:
# "<" = little endian, B = byte, f = float
# TX Velocity: Opcode (B), linear_x (f), angular_z (f)
TX_VELOCITY_FORMAT = "<Bff"


# RX IMU Payload Format:
#   "<" = little endian, B = byte, f = float, d = double
#   orientation wxyz (4f = 16B) + cov (9d = 72B) + ang_vel xyz (3f = 12B) + cov (9d = 72B) + lin_acc xyz (3f = 12B) + cov (9d = 72B)
RX_IMU_FORMAT = "<4f9d3f9d3f9d"
RX_IMU_SIZE = struct.calcsize(RX_IMU_FORMAT)


# RX GPS Payload Format:
#   "<" = little endian, d = double
#   lon (d = 8B), lat (d = 8B), alt (d = 8B), pos_cov (9d = 72B)
RX_GPS_FORMAT = "<3d9d"
RX_GPS_SIZE = struct.calcsize(RX_GPS_FORMAT)




def compute_crc8(data: bytes) -> int:
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def getPort(vid: int, pid: int) -> str:
    device_list = list_ports.comports()
    for device in device_list:
        if device.vid == vid and device.pid == pid:
            return device.device
    raise OSError(f"Device with VID {hex(vid)} and PID {hex(pid)} not found")


class MicrocontrollerCommunicatorNode(Node):
    """
    Communication layer to allow the microcontroller to access ROS topics.
    """

    def __init__(self) -> None:
        super().__init__("microcontroller_communicator")

        self.gps_position_publisher = self.create_publisher(NavSatFix, "/gps/fix", qos_profile_sensor_data)
        self.imu_data_publisher = self.create_publisher(Imu, "/imu", qos_profile_sensor_data)

        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)

        serial_port = getPort(MICROCONTROLLER_VID, MICROCONTROLLER_PID)
        self.microcontroller_serial = serial.Serial(serial_port, baudrate=BAUD_RATE, timeout=0.005)
        self.microcontroller_serial.reset_input_buffer()


        self.create_timer(1.0 / REFRESH_RATE, self.timer_callback)
        self.cmd_vel = Twist()


        # Magnetic declination state (computed automatically on first valid GPS fix)
        # The magnetic declination is just the amount we have to add to the raw heading value to make it actually accurate
        # Different longitude/ latitudes have different errors that are added to the raw imu reading from the microcontroller
        self.geomag = GeoMag()
        self.declination_radian = None



    def cmd_vel_callback(self, cmd_vel: Twist) -> None:
        self.cmd_vel = cmd_vel



    def send_velocity_command(self) -> None:
        """
        Sends forward velocity (linear.x in m/s) and yaw rate (angular.z in rad/s) for differential/skid-steer
        """
        vx = float(self.cmd_vel.linear.x)
        wz = float(self.cmd_vel.angular.z)

        body = struct.pack(TX_VELOCITY_FORMAT, OPCODE_TX_VELOCITY, vx, wz)
        checksum = compute_crc8(body)
        packet = bytes([SYNC_BYTE]) + body + bytes([checksum])

        self.microcontroller_serial.write(packet)


    def read_incoming_packets(self) -> None:
        """Reads all currently buffered packets from the microcontroller."""
        while self.microcontroller_serial.in_waiting > 0:
            sync_byte = self.microcontroller_serial.read(1)
            if not sync_byte or sync_byte[0] != SYNC_BYTE:
                continue

            opcode_byte = self.microcontroller_serial.read(1)
            if not opcode_byte:
                break

            opcode = opcode_byte[0]

            if opcode == OPCODE_RX_GPS:
                payload = self.microcontroller_serial.read(RX_GPS_SIZE)
                checksum_byte = self.microcontroller_serial.read(1)

                if len(payload) == RX_GPS_SIZE and len(checksum_byte) == 1 and checksum_byte[0] == compute_crc8(opcode_byte + payload):
                    self.parse_and_publish_gps(payload)
                else:
                    self.get_logger().warn("Malformed GPS packet")


            elif opcode == OPCODE_RX_IMU:
                payload = self.microcontroller_serial.read(RX_IMU_SIZE)
                checksum_byte = self.microcontroller_serial.read(1)

                if len(payload) == RX_IMU_SIZE and len(checksum_byte) == 1 and checksum_byte[0] == compute_crc8(opcode_byte + payload):
                    self.parse_and_publish_imu(payload)
                else:
                    self.get_logger().warn("Malformed IMU packet")


            else:
                self.get_logger().warn(f"Unknown opcode {hex(opcode)}")




    def parse_and_publish_gps(self, payload: bytes) -> None:
        unpacked = struct.unpack(RX_GPS_FORMAT, payload)

        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = "base_link"   # basically just saying that the antenna is at the true location of the robot

        gps_msg.status.status = NavSatStatus.STATUS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS

        # Protocol defines order as: longitude, latitude, altitude
        gps_msg.longitude = unpacked[0]
        gps_msg.latitude = unpacked[1]
        gps_msg.altitude = unpacked[2]

        if self.declination_radian is None and (gps_msg.latitude != 0.0 or gps_msg.longitude != 0.0):
            decimal_year = date.today().year + (date.today().timetuple().tm_yday / 365.25)
            result = self.geomag.calculate(glat=gps_msg.latitude, glon=gps_msg.longitude, alt=gps_msg.altitude / 1000.0, time=decimal_year)
            self.declination_radian = math.radians(result.d)

        # Position covariance (9 float64)
        gps_msg.position_covariance = list(unpacked[3:12])
        gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN

        self.gps_position_publisher.publish(gps_msg)


    def parse_and_publish_imu(self, payload: bytes) -> None:
        unpacked = struct.unpack(RX_IMU_FORMAT, payload)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()

         # Basically just saying that the IMU data that we are measuring is in the robots base frame of reference and not some global frame of reference
         # IE linear acceleration +x is just whatever is forward relative to where the robot is facing.
        imu_msg.header.frame_id = "base_link"

        # Orientation (w, x, y, z)
        raw_qw, raw_qx, raw_qy, raw_qz = unpacked[0], unpacked[1], unpacked[2], unpacked[3]

        # computed once we get our first GPS reading to correct the yaw
        if self.declination_radian is not None:
            imu_rotation = R.from_quat([raw_qx, raw_qy, raw_qz, raw_qw])
            declination_rotation = R.from_euler("z", -self.declination_radian, degrees=False)   # magnetic declination only applies to the yaw (aka the z euler angle)
            qx, qy, qz, qw = (declination_rotation * imu_rotation).as_quat()     # compose the rotations and then cast it to a quaternion to publish

        # if we do not have our first GPS reading, then don't publish anything related to the IMU since it won't be accurate
        else:
            return


        imu_msg.orientation.w = float(qw)
        imu_msg.orientation.x = float(qx)
        imu_msg.orientation.y = float(qy)
        imu_msg.orientation.z = float(qz)

        # Orientation covariance (9 float64)
        imu_msg.orientation_covariance = list(unpacked[4:13])

        # Angular velocity (x, y, z)
        imu_msg.angular_velocity.x = unpacked[13]
        imu_msg.angular_velocity.y = unpacked[14]
        imu_msg.angular_velocity.z = unpacked[15]

        # Angular velocity covariance (9 float64)
        imu_msg.angular_velocity_covariance = list(unpacked[16:25])

        # Linear acceleration (x, y, z)
        imu_msg.linear_acceleration.x = unpacked[25]
        imu_msg.linear_acceleration.y = unpacked[26]
        imu_msg.linear_acceleration.z = unpacked[27]

        # Linear acceleration covariance (9 float64)
        imu_msg.linear_acceleration_covariance = list(unpacked[28:37])

        self.imu_data_publisher.publish(imu_msg)



    def timer_callback(self) -> None:
        """Transmits velocity command and publishes any received sensor data."""
        self.send_velocity_command()
        self.read_incoming_packets()



    def close_serial(self) -> None:
        if hasattr(self, "microcontroller_serial") and self.microcontroller_serial.is_open:
            self.microcontroller_serial.close()



def main(args=None):
    rclpy.init(args=args)
    node = MicrocontrollerCommunicatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_serial()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == "__main__":
    main()
