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
            executable='dead_reckoning',
            name='dead_reckoning',
            namespace='robot_estimate',
            remappings=[('/wheel_displacements','/wheel_displacements_simulated')]
        ),
        Node( 
            package='mobrob',
            executable='mobile_robot_animator',
            name='animator_estimated',
            namespace='robot_estimate',
            remappings=[('/robot_pose_simulated','/robot_pose_estimated')]
        ), 
        
# Now change the "robot_control" section to use Closed-Loop Path Following, and a path that is geometric rather than time-and-wheel-speed.
# Two Nodes: "closed_loop_path_follower" and "set_path_to_follow" 
# (Note: the other control node "set_desired_wheel_speeds" is already removed above.)
# Set both of them to send output to the screen (output='screen'). 
        Node(
            package='mobrob',
            executable='closed_loop_path_follower',
            name='closed_loop_path_follower',
            namespace='robot_control',
            output='screen'
        ),
        Node(
            package='mobrob',
            executable='set_path_to_follow',
            name='set_path_to_follow',
            namespace='robot_control',
            output='screen'
        )        
    ])
