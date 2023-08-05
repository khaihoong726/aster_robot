import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from std_msgs.msg import String
from sensor_msgs.msg import Imu

GYRO_X_OFFSET = 0.062
GYRO_Y_OFFSET = -0.02
GYRO_Z_OFFSET = 0.02

class RobotImu(Node):
    def __init__(self):
        super().__init__('aster_imu_node')
        self.get_logger().info("Aster's IMU node is running")

        self.accel = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]

        self.create_subscription(
            String,
            '/receive_serial',
            self.get_serial_data,
            qos_profile = qos_profile_system_default
        )

        self.raw_imu_pub = self.create_publisher(
            Imu,
            'imu/data_raw',
            qos_profile = qos_profile_system_default
        )

        self.thread_run = True
        self.thread = threading.Thread(target=self.send_raw_imu, daemon=True)
        self.thread.start()

    def get_serial_data(self, msg):
        data = msg.data.replace("\r\n", "").split(',')
        self.accel = np.array([float(data[2]), float(data[3]), float(data[4])])
        self.gyro = np.array([float(data[5]), float(data[6]), float(data[7])])

    def send_raw_imu(self):
        while self.thread_run:
            imu = Imu()
            curr_time = self.get_clock().now().to_msg()

            imu.header.stamp = curr_time
            imu.header.frame_id = "imu_link"

            imu.orientation_covariance[0] = -1

            imu.linear_acceleration.x = self.accel[0]
            imu.linear_acceleration.y = self.accel[1]
            imu.linear_acceleration.z = self.accel[2]

            imu.linear_acceleration_covariance[0] = 0.1
            imu.linear_acceleration_covariance[4] = 0.1
            imu.linear_acceleration_covariance[8] = 0.1

            imu.angular_velocity.x = self.gyro[0] + GYRO_X_OFFSET
            imu.angular_velocity.y = self.gyro[1] + GYRO_Y_OFFSET
            imu.angular_velocity.z = self.gyro[2] + GYRO_Z_OFFSET

            if abs(self.gyro[2] + GYRO_Z_OFFSET) < 0.03:
                imu.angular_velocity.z = 0.0

            imu.angular_velocity_covariance[0] = 0.1
            imu.angular_velocity_covariance[4] = 0.1
            imu.angular_velocity_covariance[8] = 0.1

            self.raw_imu_pub.publish(imu)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    robot_imu = RobotImu()

    try:
        rclpy.spin(robot_imu)
    except:
        robot_imu.thread_run = False

if __name__ == '__main__':
    main()
