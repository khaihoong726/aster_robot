import serial
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String

class RobotSerial(Node):
    def __init__(self):
        super().__init__('aster_serial_node')
        self.get_logger().info("Aster's serial node is running")

        self.connected = False
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud_rate', 115200)

        serial_port = self.get_parameter('serial_port').value
        serial_baud_rate = self.get_parameter('serial_baud_rate').value
        try:
            self.board = serial.Serial(serial_port, serial_baud_rate)
            self.connected = True
            self.get_logger().info('Connected to Board')
        except:
            self.get_logger().error('Could not connect to Board')
            self.connected = False

        self.create_subscription(
            String,
            '/send_serial',
            self.send_serial,
            qos_profile = qos_profile_system_default
        )

        self.serial_pub = self.create_publisher(
            String,
            '/receive_serial',
            qos_profile = qos_profile_system_default
        )

        self.thread_run = True
        self.thread = threading.Thread(target=self.receive_serial, daemon=True)
        self.thread.start()

    def send_serial(self, msg):
        if self.connected:
            data = msg.data
            self.board.write(data.encode('utf-8'))

    def receive_serial(self):
        if self.connected:
            while self.thread_run:
                data = self.board.readline()
                data = data.decode('utf-8').replace('\x00','').replace('\r\n','')
                msg = String()
                msg.data = data
                self.serial_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    robot_serial = RobotSerial()

    try:
        rclpy.spin(robot_serial)
    except KeyboardInterrupt:
        robot_serial.thread_run = False
        robot_serial.board.close()

if __name__ == '__main__':
    main()
