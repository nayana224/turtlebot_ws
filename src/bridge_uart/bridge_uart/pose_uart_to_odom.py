import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
import tf2_ros
import serial
import re
import math

# 시리얼 포트 설정
SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE = 115200
pattern = re.compile(r"\[POSE\] x:(-?\d+\.\d+) y:(-?\d+\.\d+) θ:(-?\d+\.\d+)")

def quaternion_from_yaw(yaw):
    return Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw / 2.0),
        w=math.cos(yaw / 2.0)
    )

class PoseToOdom(Node):
    def __init__(self):
        super().__init__('pose_uart_to_odom')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # TF broadcaster (optional)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 시리얼 포트 오픈
        self.serial = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
        self.get_logger().info(f"Listening on {SERIAL_PORT}")

        self.create_timer(0.05, self.read_serial)

    def read_serial(self):
        if self.serial.in_waiting > 0:
            line = self.serial.readline().decode('utf-8').strip()
            match = pattern.match(line)
            if match:
                x = float(match.group(1))
                y = float(match.group(2))
                theta = float(match.group(3))

                # 오도메트리 메시지 생성
                odom = Odometry()
                odom.header.stamp = self.get_clock().now().to_msg()
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y
                odom.pose.pose.orientation = quaternion_from_yaw(theta)

                self.odom_pub.publish(odom)

                # tf 브로드캐스팅 (선택)
                t = TransformStamped()
                t.header.stamp = odom.header.stamp
                t.header.frame_id = "odom"
                t.child_frame_id = "base_link"
                t.transform.translation.x = x
                t.transform.translation.y = y
                t.transform.translation.z = 0.0
                q = quaternion_from_yaw(theta)
                t.transform.rotation = q

                self.tf_broadcaster.sendTransform(t)

                self.get_logger().info(f"Published odom: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdom()
    rclpy.spin(node)
    rclpy.shutdown()
