#!/usr/bin/env python3

# =============================================================================
# Peter G. Adamczyk 
# Updated 2024-10-11
# =============================================================================


import rclpy
from rclpy.node import Node
import numpy as np
import traceback 
from geometry_msgs.msg import Pose2D
from mobrob_interfaces.msg import ME439WheelDisplacements



    
class DeadReckoning(Node): 
    def __init__(self):
        super().__init__('dead_reckoning')
        
        # Variables for the robot's position
        self.r_center_world_estimated = np.array([0.,0.])  # Position (r) of the robot in the World frame. 
        self.theta_estimated = 0.  # heading angle (theta) of the robot relative to the World frame. 

        # Variables for the robot's wheel displacements (to keep knowledge of it from one step to the next)
        self.d_left_previous = 0.
        self.d_right_previous = 0.
        # Rate to set how often the estimated "pose" is published
        self.f = 10.     # Hz 
        #==============================================================================
        # # Get parameters from rosparam
        # # NOTE this is the Estimator, so we should use the "model" parameters. 
        # # This will enable us to compare the "simulated" robot (considered the true robot location) 
        # #  and the "estimated" robot (position estimated based on dead-reckoning)
        #==============================================================================
        self.wheel_width = self.declare_parameter('/wheel_width_model', 0.151).value 
        self.body_length = self.declare_parameter('/body_length', 0.235).value 
        self.wheel_radius = self.declare_parameter('/wheel_radius_model', 0.030).value 
    
    
        #==============================================================================
        ####    CODE HERE:
        #     # Here start a Subscriber to the "/wheel_displacements" topic.
        #     #  If you can't remember the Message type, use "ros2 topic info" 
        #     #  (this message is published by the "wheel_control" node)
        #     #  Set the function "dead_reckoning" as a callback - this is the function that will be called when a message comes in. 
        #==============================================================================
        self.sub_wheel_disps = self.create_subscription( ME439WheelDisplacements, '/wheel_displacements', self.dead_reckoning, 1 )  

            
        #==============================================================================
        #     # Here start a Subscriber to the "/robot_position_override" topic.
        #     #  Topic "/robot_position_override", message type "Pose2D"
        #     #  Call the function "set_pose" as a callback
        #     #  ** Publishing on this topic elsewhere is optional, but this gives the 
        #     #     node the ability to take a position override from e.g. an external 
        #     #     measurement or a known starting position. 
        #==============================================================================
        self.sub_position_override = self.create_subscription(Pose2D, '/robot_position_override', self.set_pose, 1)  
            
            
        #==============================================================================
        ####    CODE HERE:
        #     # Here a Publisher for the Estimated Robot Pose. 
        #     # Topic '/robot_pose_estimated', Message type: Pose2D
        #     # Also create a message to hold data that gets published on this topic. 
        #==============================================================================
        self.pub_robot_pose_estimated = self.create_publisher(  Pose2D, '/robot_pose_estimated', 1)
        self.robot_pose_estimated_message = Pose2D()

    
        # =============================================================================
        #     # Timer to set a publication rate. This calls a publication callback
        # =============================================================================
        self.pub_timer = self.create_timer(1.0/self.f, self.pub_callback)
    
    # =============================================================================
    #     # Callback to do the publication
    #     # This function is called by the timer above at a constant rate. Its job is to publish the current estimate of the robot pose. 
    # =============================================================================
    def pub_callback(self): 
        # Pack the message
        self.robot_pose_estimated_message.x = self.r_center_world_estimated[0]
        self.robot_pose_estimated_message.y = self.r_center_world_estimated[1]
        self.robot_pose_estimated_message.theta = self.theta_estimated
        # Publish the pose
        self.pub_robot_pose_estimated.publish(self.robot_pose_estimated_message)
        
        
    # =============================================================================
    # # Callback function for "dead-reckoning" (alternatively called "odometry")
    # =============================================================================
    def dead_reckoning(self, msg_in): 
    ####    CODE HERE: extract the wheel displacements from the message in variable msg_in. 
        # REPLACE the zeros with the proper expressions. 
        # Look in the message file for ME439WheelDisplacements.msg to find the variable 
        # names for left and right wheel displacements. 
        # Or, just ask ROS: "ros2 interface show mobrob_interfaces/msg/ME439WheelDisplacements"
        # Syntax is msg_in.variable_name
        d_left = msg_in.disp0
        d_right = msg_in.disp1
        
    ####    CODE HERE: Compute the CHANGE in displacement of each wheel
        # Old values are in self.d_left_previous and self.d_right_previous.        
        # REPLACE the zeros with the proper expressions. 
        diff_left = d_left - self.d_left_previous
        diff_right = d_right - self.d_right_previous
        
    ####    CODE HERE: STORE the new values of d_left and d_right for the next call
        # REPLACE the zeros with the proper expressions. 
        self.d_left_previous = d_left
        self.d_right_previous = d_right
        
    ####    CODE HERE: compute change in path length and change in angle
        # REPLACE the zeros with the proper expressions (see lecture notes). 
        # use "diff_left" and "diff_right" which were set a few lines above. 
        diff_pathlength = (diff_left + diff_right)/2
        diff_theta = (diff_right - diff_left)/self.wheel_width

    ####    CODE HERE: compute the AVERAGE heading angle (theta) during the movement 
        # That's halfway between the old angle and the new.
        # This makes the dead-reckoning more accurate than using just the old theta or the new one. 
        theta_avg = self.theta_estimated + diff_theta/2
        
    ####    CODE HERE: compute the change in position and heading according to the dead-reckoning equations
        # REPLACE the zeros with the proper expressions (see lecture notes). 
        # Remember that sine and cosine are in the "numpy" package, which has been imported as "np"
        self.r_center_world_estimated[0] = self.r_center_world_estimated[0] + -np.sin(theta_avg)*diff_pathlength      # x-direction position
        self.r_center_world_estimated[1] = self.r_center_world_estimated[1] + np.cos(theta_avg)*diff_pathlength      # y-direction position
        self.theta_estimated = self.theta_estimated + diff_theta

    #==============================================================================
    #     End of function "dead_reckoning"
    #==============================================================================
    
    
    
    # =============================================================================
    # # Callback function for "set_pose" (to override the estimated position)
    # # This node receives a "Pose2D" message type, which should be used to replace 
    # # the current estimated pose. 
    # =============================================================================
    def set_pose(self, msg_in): 
    #    CODE HERE: extract the pose from the message file. 
        # Look in the message file for ME439WheelDisplacements.msg to find the variable names for left and right wheel displacements. 
        # Or "ros2 interface show mobrob_interfaces/msg/ME439WheelDisplacements"
        self.r_center_world_estimated[0] = msg_in.x
        self.r_center_world_estimated[1] = msg_in.y
        self.theta_estimated = msg_in.theta
            
    #==============================================================================
    #     # End of function "set_pose"
    #==============================================================================
    
    
    
def main(args=None): 
    try: 
        rclpy.init(args=args)
        dead_reckoning_instance = DeadReckoning()
        rclpy.spin(dead_reckoning_instance)
    
    except: 
        traceback.print_exc()
    
    
if __name__ == '__main__':
    main()
