import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
import tf2_ros
import math
import time

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_odom_publisher')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.br = tf2_ros.TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vth = 0.0
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def cmd_callback(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def timer_callback(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9

        delta_x = self.vx * math.cos(self.th) * dt
        delta_y = self.vx * math.sin(self.th) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        self.last_time = current_time

        # quaternion 변환
        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        # Odometry 메시지
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "1st_floor"  # base_link 이름

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth

        self.pub.publish(odom)

        # TF 브로드캐스트
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "1st_floor"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = OdometryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
