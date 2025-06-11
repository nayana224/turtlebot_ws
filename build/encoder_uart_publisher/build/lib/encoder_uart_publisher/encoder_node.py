# encoder_node.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')

        # UART 포트 설정 (라즈베리파이에 맞게 수정 필요)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

        # 토픽 퍼블리셔 생성
        self.publisher_ = self.create_publisher(String, 'encoder_data', 10)

        # 타이머 (0.1초마다 읽기)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').strip()
            msg = String()
            msg.data = line
            self.publisher_.publish(msg)
            self.get_logger().info(f'UART 수신: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
