
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open('/tmp/robot.urdf').read()
            }]
        ),
        Node(
            package='keyboard_sim',
            executable='fake_diff_drive_node',
            name='fake_diff_drive_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str('/tmp/robot_sim.rviz')],
            output='screen'
        )
    ])
