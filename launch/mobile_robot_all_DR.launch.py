from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mobrob',
            executable='sensors_raw',
            name='sensors_raw',
            namespace='robot_operate'
        ),
        Node(
            package='mobrob',
            executable='sensors_processor',
            name='sensors_processor',
            namespace='robot_operate'
        ),
        Node(
            package='mobrob',
            executable='wheel_control',
            name='wheel_control',
            namespace='robot_operate'
        ),
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
        ),
        
# "dead_reckoning" node in the namespace "robot_estimate" 
        Node( 
            package='mobrob',
            executable='dead_reckoning',
            name='dead_reckoning',
            namespace='robot_estimate'
        ),

# Then add a "mobile_robot_animator" node for the Estimated state
# * name it "animator_estimated" and put it in the "robot_estimate" namespace . 
# * Remap to make it subscribe to "/robot_pose_estimated" instead of "/robot_pose_simulated". 
        Node( 
            package='mobrob',
            executable='mobile_robot_animator',
            name='animator_estimated',
            namespace='robot_estimate',
            remappings=[('/robot_pose_simulated','/robot_pose_estimated')]
        )
    ])
