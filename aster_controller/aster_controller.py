import threading
import time
from math import pi

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .helper.differential_drive import DifferentialDriveRobot
from .helper.encoder import Encoder
from .helper.pid import PidController

class RobotController(Node):
    def __init__(self):
        super().__init__('aster_controller_node')
        self.get_logger().info("Aster's controller node is running")

        self.declare_parameter('wheel_base', 0.138) # 0.168
        self.declare_parameter('wheel_radius', 0.0325) # 0.04
        self.declare_parameter('Kp', 2.5)
        self.declare_parameter('Ki', 1.25)
        self.declare_parameter('Kd', 0.001)
        self.declare_parameter('delta_time', 0.1)
        self.declare_parameter('enc_ticks_per_rev', 20) # 35
        self.declare_parameter('left_base_speed', 0)
        self.declare_parameter('right_base_speed', 0)

        wheel_base = self.get_parameter('wheel_base').value
        wheel_radius = self.get_parameter('wheel_radius').value
        self.robot = DifferentialDriveRobot(wheel_base, wheel_radius)
        self.left_encoder_data = 0
        self.right_encoder_data = 0

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.get_velocities,
            qos_profile=qos_profile_system_default
        )

        self.create_subscription(
            String,
            '/receive_serial',
            self.get_encoder_data,
            qos_profile=qos_profile_system_default
        )

        self.send_serial_pub = self.create_publisher(
            String,
            '/send_serial',
            qos_profile=qos_profile_system_default
        )

        self.thread_run = True
        self.thread = threading.Thread(target=self.send_velocities, daemon=True)
        self.thread.start()

    def get_velocities(self, msg):
        self.robot.linear_velocity = msg.linear.x
        self.robot.angular_velocity = msg.angular.z

    def get_encoder_data(self, msg):
        data = msg.data.split(',')
        self.left_encoder_data = int(data[0])
        self.right_encoder_data = int(data[1])

    def send_velocities(self):
        Kp = self.get_parameter('Kp').value
        Ki = self.get_parameter('Ki').value
        Kd = self.get_parameter('Kd').value
        delta_time = self.get_parameter('delta_time').value
        enc_ticks_per_rev = self.get_parameter('enc_ticks_per_rev').value
        left_base_speed = self.get_parameter('left_base_speed').value
        right_base_speed = self.get_parameter('right_base_speed').value

        left_encoder = Encoder(enc_ticks_per_rev)
        right_encoder = Encoder(enc_ticks_per_rev)

        left_PID = PidController(Kp, Ki, Kd, delta_time, False)
        left_PID.set_output_limits(0, 255)

        right_PID = PidController(Kp, Ki, Kd, delta_time, False)
        right_PID.set_output_limits(0, 255)

        while self.thread_run:
            left_encoder.update(self.left_encoder_data)
            right_encoder.update(self.right_encoder_data)

            measured_left_ang_vel = ((left_encoder.increment / enc_ticks_per_rev) * 2 * pi) / delta_time
            measured_right_ang_vel = ((right_encoder.increment / enc_ticks_per_rev) * 2 * pi) / delta_time

            desired_left_ang_vel, desired_right_ang_vel = self.robot.compute_wheel_angular_velocities()

            if desired_left_ang_vel < 0:
                left_encoder.direction = -1
            else:
                left_encoder.direction = 1

            if desired_right_ang_vel < 0:
                right_encoder.direction = -1
            else:
                right_encoder.direction = 1

            desired_left_ang_vel = abs(desired_left_ang_vel)
            desired_right_ang_vel = abs(desired_right_ang_vel)

            left_PID.set_setpoint(desired_left_ang_vel)
            left_PID.set_input(measured_left_ang_vel)

            right_PID.set_setpoint(desired_right_ang_vel)
            right_PID.set_input(measured_right_ang_vel)

            left_PID.compute()
            right_PID.compute()

            left_base_speed = left_encoder.direction * left_PID.output
            right_base_speed = right_encoder.direction * right_PID.output

            if desired_left_ang_vel == 0.0:
                left_base_speed = 0
            if desired_right_ang_vel == 0.0:
                right_base_speed = 0

            msg = String()
            msg.data = "{} {}\r\n".format(int(left_base_speed), int(right_base_speed))
            self.send_serial_pub.publish(msg)
            time.sleep(delta_time)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        robot_controller.thread_run = False

if __name__ == '__main__':
    main()
