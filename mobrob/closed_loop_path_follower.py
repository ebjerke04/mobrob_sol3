#!/usr/bin/env python3

# =============================================================================
# Peter G. Adamczyk 
# 2018-10
# Updated 2025-03-04
# =============================================================================

import rclpy
from rclpy.node import Node
import rclpy.action, rclpy.node, rclpy.executors
from rclpy.callback_groups import ReentrantCallbackGroup #, MutuallyExclusiveCallbackGroup 
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor

import traceback 
import numpy as np
# IMPORT the custom message: 
# we import it "from" the ROS package we created it in (here "mobrob_interfaces") with an extension of .msg ...
# and actually import the message types by name 
from mobrob_interfaces.msg import ME439WheelSpeeds, ME439PathSpecs 
from geometry_msgs.msg import Pose2D
import mobrob.me439_mobile_robot_class_v02 as m439rbt  # REMEMBER to call the right file (version, or use the _HWK if needed)

# Import an Action definition
from mobrob_interfaces.action import ME439FollowPath
# Two utilities for making the whole program work. 
import threading # Allows multiple Threads within one program. 
import time # allows a blocking "sleep" call - could be replaced by node.create_rate(f_Hz) at some point


# =============================================================================
#     Set up a closed-loop path controller
#     Use an Action to actually do the path following. 
#     Subscribe to "robot_pose_estimated" (Pose2D) for updates on robot position. 
#     and Publish "wheel_speeds_desired" (ME439WheelSpeeds)
# =============================================================================


class ClosedLoopPathFollower(Node):
    def __init__(self):
        super().__init__('closed_loop_path_follower')
        
        self._goal_handle = None
        self._goal_lock = threading.Lock()     
        
        ####    CODE HERE: Create Subscriber and Publisher as described
        # Create a Subscriber to the robot's current estimated position '/robot_pose_estimated', message type Pose2D
        # with a callback to "self.update_pose"
        self.sub_robot_pose_estimated = self.create_subscription(Pose2D, '/robot_pose_estimated', self.update_pose, 1)
        
        # Create a Subscriber to updated path specs '/path_segment_spec', message type ME439PathSpecs
        # with a callback to "self.update_path"
        self.sub_robot_pose_estimated = self.create_subscription(ME439PathSpecs, '/path_segment_spec', self.update_path, 1)
        
        # Create a publisher for the desired wheel speeds. Name the topic "/wheel_speeds_desired", with message type "ME439WheelSpeeds"
        self.pub_wheel_speeds = self.create_publisher(ME439WheelSpeeds, '/wheel_speeds_desired', 1)
        ####    CODE END
        
        # Create an Action Server to follow the designated path segment
        # With a callback to "set_path"
        # Note several additional forms of callback to allow this ActionServer to nicely handle a sequence of callbacks. 
        self.CL_action_server = ActionServer(
            self,
            ME439FollowPath,
            'follow_path',
            execute_callback=self.set_path,
            callback_group = ReentrantCallbackGroup(),
            goal_callback = self.goal_callback, 
            handle_accepted_callback = self.handle_accepted_callback, 
            cancel_callback = self.cancel_callback)
        
        # Set an "initializing" variable to prevent path following before the node gets a path. 
        self.initializing = True
            
        # Create state variables. 
        self.estimated_pose = Pose2D()
        
        # Get parameters from rosparam
        self.wheel_width = self.declare_parameter('/wheel_width_model', 0.151).value 
        self.body_length = self.declare_parameter('/body_length', 0.235).value 
        self.wheel_radius = self.declare_parameter('/wheel_radius_model', 0.030).value 

        # Create a mobile robot object from the Imported module "me439_mobile_robot_class"
        self.robot = m439rbt.robot(self.wheel_width, self.body_length, self.wheel_radius)

        ####    CODE HERE
        # Add Parameters for Closed-loop controller parameters: Vmax, Beta, gamma, angle_focus_factor, forward_only
        # Give them default values, respectively: e.g. 0.2, 1.5, 1.5, 0.0, False (not forward_only)
        self.Vmax = self.declare_parameter('/Vmax',0.2).value 
        self.Beta = self.declare_parameter('/Beta',1.5).value 
        self.gamma = self.declare_parameter('/gamma',1.5).value 
        self.angle_focus_factor = self.declare_parameter('/angle_focus_factor',0.0).value 
        self.forward_only = self.declare_parameter('/forward_only',False).value 
        ####    END CODE

        # Zero the speeds to start
        msg_speeds = ME439WheelSpeeds()
        msg_speeds.v0 = 0.0
        msg_speeds.v1 = 0.0
        self.pub_wheel_speeds.publish(msg_speeds)
        # self.get_logger().info(msg_speeds)    
        
    # Callback to handle new incoming Goal requests. 
    # Note: here it just gives ACCEPT response to all requests. The other option is to REJECT them if desired. 
    # Those are both just for bookkeeping. If you ACCEPT, that sends a signal to the Action Client and also calls the "handle_accepted_callback" below to actually do something with it.
    def goal_callback(self, goal_request:ME439FollowPath):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT
    
    # Callback to handle newly Accepted Goal requests. 
    # This is set up as a single-goal system: if a new one is accepted, the old one has to be aborted. 
    def handle_accepted_callback(self, goal_handle:rclpy.action.server.ServerGoalHandle):
        self.get_logger().info('entered Handle Accepted Callback')
        
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            # Set a retained copy of the requested goal 
            self._goal_handle = goal_handle
            # Set the path that was conveyed with it. 
            self.path_segment_spec = self._goal_handle.request
        
        # Here actually send the goal to the "execution" callback down below. 
        self.get_logger().info('Starting new goal')
        goal_handle.execute()
    

    # Callback to handle Cancel requests. 
    # Note: here it gives an ACCEPT response to all cancel requests. 
    def cancel_callback(self, goal_handle:rclpy.action.server.ServerGoalHandle):
        self.get_logger().info('Cancel request')
        return CancelResponse.ACCEPT
    
    
    # "execution" Callback - sets a new path goal and monitors progress, publishing feedback. 
    # Also sets "succeed" when completed.  
    def set_path(self, goal_handle): 
        # First assign the incoming action's goal request contents
        # self._goal_handle = goal_handle
        self.get_logger().info('setting new goal')
        self.fraction_complete = 0.
        self.initialize_psi_world= True
        self.estimated_psi_world_previous = 0.         
        self.initializing = False

        while self.fraction_complete < 1.0 and goal_handle.is_active:
            # Publish Feedback: Fraction Complete
            feedback_msg = ME439FollowPath.Feedback()
            feedback_msg.fraction_complete = self.fraction_complete
            goal_handle.publish_feedback(feedback_msg)        # Publish Feedback
            time.sleep(0.1)  # Could perhaps be replaced with a node.create_rate(f_Hz) object. 
        
        # # Finally, if we get here, the path segment is complete. 
        with self._goal_lock:
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return ME439FollowPath.Result()

            # Declare success in the Action  
            goal_handle.succeed()
        # And Return the Result
        result = ME439FollowPath.Result()
        result.fraction_complete = self.fraction_complete
        # self.get_logger().info('sending result')
        return result        
    
    
    # =============================================================================
    # Subscriber Callback to update the pose and compute control
    # This is not part of the Action server - it subscribes to an ordinary topic and uses that as a trigger to compute control. 
    # As part of the operation, it sets desired wheel speeds and publishes those. 
    # =============================================================================
    def update_pose(self, pose_msg_in):
        # assign the incoming message 
        self.estimated_pose = pose_msg_in
        
        # If there is no path_spec yet, do nothing (cannot follow it)
        if self.initializing: 
            return
        
        # First assign the action's goal request contents
        # path_segment_spec = self._goal_handle.request
        path_segment_spec = self.path_segment_spec
        fraction_complete = self.fraction_complete 

        if fraction_complete < 1.0:
            pose_xy = np.array([self.estimated_pose.x, self.estimated_pose.y])
            pose_theta = np.array([self.estimated_pose.theta])
            if np.isinf(path_segment_spec.length):  # if the length is infinite, that's an error. Cancel the specified path. 
                self.get_logger().info('Closed-loop path following: Path length cannot be infinite. Aborting goal.')
                self.path_goal.abort()
                return; 
            
            # Create a vector variable for the Origin of the path segment 
            path_segment_Origin = np.array([path_segment_spec.x0, path_segment_spec.y0] )

            # Forward distance relative to a Line path is computed along the Forward axis. 
            # Therefore Define the Forward Axis: 
            path_segment_y0_vector = np.array([-np.sin(path_segment_spec.theta0), np.cos(path_segment_spec.theta0)])
            # local X is computed perpendicular to the segment. 
            # Therefore define the Perpendicular axis.
            path_segment_x0_vector = np.array([-np.sin(path_segment_spec.theta0 - np.pi/2.0), np.cos(path_segment_spec.theta0 - np.pi/2.0)])
            # Define path curvature -- this will be approximately zero for the straight line case. 
            path_segment_curvature = 1.0/path_segment_spec.radius
            
            
        # =============================================================================
        #     ### First Compute the robot's position relative to the path (x, y, theta)
        #     ### and the local path properties (curvature 1/R and segment completion percentage)
        # =============================================================================
                
            # If it is a Line (Infinite radius)
            if np.isinf(path_segment_spec.radius):
                path_segment_endpoint = path_segment_Origin + path_segment_spec.length*path_segment_y0_vector
                
                # compute position relative to Segment: 
                estimated_xy_rel_to_segment_origin = (pose_xy - path_segment_Origin)   # XY vector from Origin of segment to current location of robot. 
                # Projection of vector from path origin to current position 
                estimated_segment_forward_pos = estimated_xy_rel_to_segment_origin.dot(path_segment_y0_vector)
                # The fraction completion can be estimated as the path length the robot has gone through, as a fraction of total path length on this segment
                fraction_complete = estimated_segment_forward_pos / path_segment_spec.length
                # Position of the robot to the Right of the segment Origin
                estimated_segment_rightward_pos = estimated_xy_rel_to_segment_origin.dot(path_segment_x0_vector)
                
                estimated_y_local = 0.0   # y_local = 0 by definition: Local coords are defined relative to the closest point on the path. 
                estimated_x_local = estimated_segment_rightward_pos
                estimated_theta_local = pose_theta - path_segment_spec.theta0 
            
            # Arc
            else:   
                curve_sign = np.sign(path_segment_spec.radius)
                path_segment_circle_center = path_segment_Origin + path_segment_spec.radius*-path_segment_x0_vector
                # determine the angular displacement of this arc. SIGNED quantity! 
                path_segment_angular_displacement = path_segment_spec.length/path_segment_spec.radius
                path_segment_ThetaEnd = path_segment_spec.theta0 + path_segment_angular_displacement
                estimated_xy_rel_to_circle_center = (pose_xy - path_segment_circle_center)
                
                # Compute angle of a vector from circle center to Robot, in the world frame, relative to the +Yworld axis. 
                # Note how this definition affects the signs of the arguments to "arctan2"
                estimated_psi_world = np.arctan2(-estimated_xy_rel_to_circle_center[0], estimated_xy_rel_to_circle_center[1])
                # unwrap the angular displacement
                if self.initialize_psi_world:
                    self.estimated_psi_world_previous = estimated_psi_world
                    self.initialize_psi_world = False
                while estimated_psi_world - self.estimated_psi_world_previous > np.pi: # was negative, is now positive --> should be more negative. 
                    estimated_psi_world += -2.0*np.pi
                while estimated_psi_world - self.estimated_psi_world_previous <= -np.pi: # was positive and is now negative --> should be more positive. 
                    estimated_psi_world += 2.0*np.pi
                
                # update the "previous angle" memory. 
                self.estimated_psi_world_previous = estimated_psi_world
                # The local path forward direction is perpendicular to this World frame origin-to-robot angle. Sign depends on left or right turn. 
                estimated_path_theta = estimated_psi_world + np.pi/2.0*curve_sign
                # The fraction completion can be estimated as the path angle the robot has gone through, as a fraction of total angular displacement on this segment
                fraction_complete = (estimated_path_theta - path_segment_spec.theta0) / path_segment_angular_displacement

                estimated_y_local = 0.0  # by definition of local coords
                # x_local is positive Inside the circle for Right turns, and Outside the circle for Left turns
                estimated_x_local = curve_sign*(np.sqrt(np.sum(np.square(estimated_xy_rel_to_circle_center))) - np.abs(path_segment_spec.radius) ) 
                estimated_theta_local = pose_theta - estimated_path_theta
            
            ## Whether Line or Arc, update the "local" coordinate state and path properties: 
            estimated_theta_local = m439rbt.fix_angle_pi_to_neg_pi(estimated_theta_local)


            # =============================================================================
            #     ### CONTROLLER for path tracking based on local position and curvature. 
            #     # parameters for the controller are 
            #     #   Vmax: Maximum allowable speed,  
            #     # and controller gains:
            #     #   Beta (gain on lateral error, mapping to lateral speed)
            #     #   gamma (gain on heading error, mapping to rotational speed)
            #     # and a control variable for the precision of turns, 
            #     #   angle_focus_factor
            # =============================================================================
                
            ####    CODE HERE:  Put in formulas for anything that is 0.0, and try the "TRY THIS" variations. 
            # First set the speed with which we want the robot to approach the path
            xdot_local_desired = -self.Beta*estimated_x_local   # Use formula from Lecture
            # limit it to +-Vmax
            xdot_local_desired = np.min([np.abs(xdot_local_desired),abs(self.Vmax)])*np.sign(xdot_local_desired)
            
            # Next set the desired theta_local 
            theta_local_desired = np.arcsin(-xdot_local_desired/self.Vmax)   # Use formula from Lecture
                    
            ## Next SET SPEED OF ROBOT CENTER. 
            ## G. Cook 2011 says just use constant speed all the time,
            ## TRY THIS FIRST
            Vc = self.Vmax    

            ## But, that drives farther from the path at first if it is facing away. 
            ## This FIX causes the speed to fall to zero if the robot is more than 90 deg from the heading we want it to have. 
            ##   The parameter "angle_focus_factor" can make it even more restrictive if needed (e.g. angle_focus_factor = 2 --> 45 deg limit). 
            ##   Value of 1.0 uses a straight cosine of the angle. 
            theta_local_error = m439rbt.fix_angle_pi_to_neg_pi(theta_local_desired - estimated_theta_local)
            if abs(theta_local_error)*self.angle_focus_factor <= np.pi :
                Vc = float(self.Vmax * np.cos(self.angle_focus_factor * m439rbt.fix_angle_pi_to_neg_pi(theta_local_desired - estimated_theta_local)))
            else: 
                Vc = 0
            
            ## Could also limit it to only forward: parameter "/forward_only"
            if self.forward_only:
                Vc = np.max([Vc,0])
            
            # Finally set the desired angular rate
            estimated_theta_local_error = m439rbt.fix_angle_pi_to_neg_pi(theta_local_desired - estimated_theta_local)
            omega = self.gamma*estimated_theta_local_error + Vc*(path_segment_curvature/(1.0+path_segment_curvature*estimated_x_local))*np.cos(estimated_theta_local)
            ####    CODE END    
            
            # Finally, use the "robot" object created elsewhere (member of the me439_mobile_robot_xx class) to translate omega and Vc into wheel speeds
            self.robot.set_wheel_speeds_from_robot_velocities(Vc, omega)
            
            # Now Publish the desired wheel speeds
            msg_speeds = ME439WheelSpeeds()
            msg_speeds.v0 = float(self.robot.left_wheel_speed)
            msg_speeds.v1 = float(self.robot.right_wheel_speed)
            self.pub_wheel_speeds.publish(msg_speeds)
            
            # Update the memory of "fraction_complete"
            self.fraction_complete = fraction_complete
        
        # If the path "fraction_complete" is greater than 1, we end the path and stop the wheels.  
        else:
             # Stop wheels
            msg_speeds = ME439WheelSpeeds()
            msg_speeds.v0 = 0.0
            msg_speeds.v1 = 0.0
            self.pub_wheel_speeds.publish(msg_speeds) 

    # =============================================================================
    # Subscriber Callback to update the Path 
    # This is not part of the Action server - it subscribes to an ordinary topic and uses that to update the path. 
    # =============================================================================
    def update_path(self,path_msg_in):
        self.path_segment_spec = path_msg_in  # This might FAIL because the first is part of an Action Request and the second is a Message of type ME439PathSpecs. If it fails, pass the individual variables one-by-one (x0, y0, theta0, radius, length. # And later maybe change the Request definition to be of type ME439PathSpecs? 
        self.get_logger().info('New path received - updated')


def main(args=None):
    rclpy.init(args=args)
    closed_loop_path_follower = ClosedLoopPathFollower()
    executor = MultiThreadedExecutor()
    rclpy.spin(closed_loop_path_follower,executor=executor)
    

if __name__ == '__main__':
    main() 
