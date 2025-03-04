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
        ),
        
# Launch some nodes in a new "namespace" (a group) called "robot_estimate"
# You will launch the "dead_reckoning" node to estimate the robot's state from sensor data, and another "animator" to draw where the robot thinks it is. 
# First launch the node "dead_reckoning" node. 
# * Remember to put a comma after the one above -- we are continuing an existing comma-separated list. 
# * Give it a "namespace" argument set to "robot_estimate" 
# * "dead_reckoning" is designed for the real robot, so it subscribes to "/robot_wheel_displacements" - which comes from the Encoders.
# In this case, we're using it with simulated wheel_displacements data because this launch file is a simulation only. 
# Therefore, REMAP topics so dead_reckoning subscribes to "/wheel_displacements_simulated" (from simulation) instead of "/wheel_displacements" (from real encoders). 
# Use this reference and search for "remapping": https://docs.ros.org/en/rolling/How-To-Guides/Launch-file-different-formats.html
        Node( 
            package='mobrob',
            executable='dead_reckoning',
            name='dead_reckoning',
            namespace='robot_estimate',
            remappings=[('/wheel_displacements','/wheel_displacements_simulated')]
        ),

# Then add a "mobile_robot_animator" node.
# * name it "animator_estimated"
# * Put it in the "robot_estimate" namespace also. 
# * Remap to make it subscribe to "/robot_pose_estimated" instead of "/robot_pose_simulated". 
        Node( 
            package='mobrob',
            executable='mobile_robot_animator',
            name='animator_estimated',
            namespace='robot_estimate',
            remappings=[('/robot_pose_simulated','/robot_pose_estimated')]
        )
        
    ])
