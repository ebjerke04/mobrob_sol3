from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobrob',
            executable='sensors_raw',
            name='sensors_raw'
        ),
        Node(
            package='mobrob',
            executable='sensors_processor',
            name='sensors_processor'
        ),
        Node(
            package='mobrob',
            executable='wheel_control',
            name='wheel_control'
        ),
        Node(
            package='mobrob',
            executable='set_desired_wheel_speeds',
            name='set_desired_wheel_speeds'
        )
    ])

