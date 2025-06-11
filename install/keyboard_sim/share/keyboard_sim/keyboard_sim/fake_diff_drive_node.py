
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
import tf2_ros
import math

class FakeDiffDrive(Node):
    def __init__(self):
        super().__init__('fake_diff_drive')
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_time = self.get_clock().now()

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.create_timer(0.05, self.timer_callback)

    def cmd_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.w * dt

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = '1st_floor'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        q = tf2_ros.transformations.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = FakeDiffDrive()
    rclpy.spin(node)
    rclpy.shutdown()
