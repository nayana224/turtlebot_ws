import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import serial
import threading
import math
import re
from transformations import quaternion_from_euler
import tf2_ros
from geometry_msgs.msg import TransformStamped

CPR = 1304.0
WHEEL_RADIUS = 0.033  # 바퀴 반지름 [m]
WHEEL_BASE = 0.160    # 바퀴 간 거리 [m]
SERIAL_PORT = '/dev/ttyUSB0'
BAUDRATE = 115200
pattern = re.compile(r"\[ENC_TOTAL\] L:(-?\d+) R:(-?\d+)")

class EncoderOdomNode(Node):
    def __init__(self):
        super().__init__('encoder_odom_node')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # 초기 pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_left = 0
        self.prev_right = 0
        self.first_read = True

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 시리얼 포트 설정
        self.ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)

        # 수신 쓰레드
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def read_loop(self):
        while True:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                match = pattern.match(line)
                if match:
                    left = int(match.group(1))
                    right = int(match.group(2))

                    if self.first_read:
                        self.prev_left = left
                        self.prev_right = right
                        self.first_read = False
                        continue

                    dl = (left - self.prev_left) * (2 * math.pi * WHEEL_RADIUS) / CPR
                    dr = (right - self.prev_right) * (2 * math.pi * WHEEL_RADIUS) / CPR
                    self.prev_left = left
                    self.prev_right = right

                    dx = (dr + dl) / 2.0
                    dtheta = (dr - dl) / WHEEL_BASE

                    self.theta += dtheta
                    self.x += dx * math.cos(self.theta)
                    self.y += dx * math.sin(self.theta)

                    self.publish_odom()
            except Exception as e:
                self.get_logger().warn(f'Error parsing: {e}')

    def publish_odom(self):
        now = self.get_clock().now().to_msg()

        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.odom_pub.publish(odom_msg)

        # TF 변환도 함께 방송
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
