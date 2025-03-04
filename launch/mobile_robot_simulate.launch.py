from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobrob',
            executable='mobile_robot_kinematic_simulator',
            name='mobile_robot_simulator',
            namespace='robot_simulate'
        ),
        Node(
            package='mobrob',
            executable='mobile_robot_animator',
            name='mobile_robot_animator',
            namespace='robot_simulate'
        ),
        Node(
            package='mobrob',
            executable='set_desired_wheel_speeds',
            name='set_desired_wheel_speeds',
            namespace='robot_control'
        )
    ])
