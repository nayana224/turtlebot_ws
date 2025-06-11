import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import time

class CmdVelToJoints(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_joints')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        self.left_pos = 0.0
        self.right_pos = 0.0
        self.left_vel = 0.0
        self.right_vel = 0.0

    def cmd_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z
        wheel_distance = 0.2  # 양 바퀴 간 거리 [m]
        wheel_radius = 0.033  # 바퀴 반지름 [m]

        # 왼쪽/오른쪽 바퀴 속도 [rad/s]
        self.left_vel = (linear - angular * wheel_distance / 2) / wheel_radius
        self.right_vel = (linear + angular * wheel_distance / 2) / wheel_radius

    def publish_joint_states(self):
        now = self.get_clock().now().to_msg()
        dt = 0.1

        self.left_pos += self.left_vel * dt
        self.right_pos += self.right_vel * dt

        js = JointState()
        js.header.stamp = now
        js.name = ['left_wheel', 'right_wheel']
        js.position = [self.left_pos, self.right_pos]
        js.velocity = [self.left_vel, self.right_vel]
        self.pub.publish(js)

def main():
    rclpy.init()
    node = CmdVelToJoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
