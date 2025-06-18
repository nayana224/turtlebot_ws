import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelToUART(Node):
    def __init__(self):
        super().__init__('cmdvel_to_uart')
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)  # USB 연결 확인 필요
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        command = f"{linear:.2f},{angular:.2f}\n"
        self.serial_port.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent to STM32: {command.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToUART()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
