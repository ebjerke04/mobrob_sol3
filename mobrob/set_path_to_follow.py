#!/usr/bin/env python3

# =============================================================================
# Peter G. Adamczyk 
# 2018-10
# Updated 2024-10-17
# =============================================================================

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import traceback 
import numpy as np

# Import the mobiile robot model
# REMEMBER to call the right file (version, or use the _HWK if needed)
import mobrob.me439_mobile_robot_class_v02 as m439rbt  

# Import an Action definition
from mobrob_interfaces.action import ME439FollowPath

from action_msgs.msg import GoalStatus

### To Be Fixed Later - made into Parameters
# Get parameters 
wheel_width = 0.151 # All you have when planning is a model - you never quite know the truth! 
body_length = 0.235
wheel_radius = 0.030

# Create a mobile robot object from the Imported module "me439_mobile_robot_class"
robot = m439rbt.robot(wheel_width, body_length, wheel_radius)


###############################################################################
# PATH_SPECS for a Closed-Loop path-following controller.
# Each segment is defined as an Arc with parameters: 
# (Xorigin, Yorigin, Theta_init, Rsigned, Length)
#-   Xorigin and Yorigin are in the World frame
#-   Theta_init is in the World frame, measuring the angle of the intended ROBOT FORWARD 
#     direction as a +z rotation from the +Yworld axis
#-   Rsigned should be + for Left turn, - for Right turn, or +/- infinity (np.inf (numpy.inf)) for a Line
#-   Length is a path Arclength for this segment. (for a Line, the line's length)
###############################################################################
# SEVERAL EXAMPLES of Direct path programming: 
# the last one not commented will be executed. 
###############################################################################
## 1 m circle starting at [0.5,0.5]
#path_specs = np.array([[0.5,0.5,0,1,2*np.pi*1]])
## 1 m circle starting at [-2,-2]
#path_specs = np.array([[-2,-2,0,1,2*np.pi*1]])
# # Picture of a cart with two wheels
# path_specs = np.array([[0,0,0,np.inf,1],[0,1,-np.pi/2,np.inf,3], [3,1,-np.pi,np.inf,1], [3,0,np.pi/2,np.inf,0.5],[2.5,0,np.pi/2,0.5,2*np.pi*0.5],[2.5,0,np.pi/2,np.inf,2],[0.5,0,np.pi/2,0.5,2*np.pi*0.5], [0.5,0,np.pi/2,np.inf,0.5]])

###############################################################################
# ALTERNATIVELY, use Geometric Constructors to automatically make the specs: 
# These functions are in "mobile_robot_class_xx.py": 
#   specify_line(x0,y0,xf,yf)
#   specify_arc(x0,y0,xf,yf,R,way='short')     # or way='long' (do you want the "short way" or the "long way" around the circle?)
###############################################################################
## 1 m circle starting at [0.5,0.5], using geometric constructor functions. 
# NOTE that a full circle is a degenerate case, so it has to be done in two halves: 
path_specs = np.array([robot.specify_arc(0.5,0.5,2.5,0.5,-1,way='long'),robot.specify_arc(2.5,0.5,0.5,0.5,-1,way='long')] )
## 1 m circle starting at [-2,-2], using geometric constructor functions. 
## NOTE that a full circle is a degenerate case, so it has to be done in two halves: 
#path_specs = np.array([robot.specify_arc(-2,-2,0,-2,-1,way='long'),robot.specify_arc(0,-2,-2,-2,-1,way='long')] )
## Arc of radius 1 m from [0.3, -0.2] to [0,0.3] and then line back to [0,0]:
#path_specs = np.array([robot.specify_arc(0.3,-0.2,0.0,0.3,1.0,way='short'), robot.specify_line(0.0,0.3,0.0,0.0)])

###############################################################################
## Interesting "bug" demos: STUDENTS: THINK ABOUT WHY THESE BEHAVE ODDLY (if they do):  
###############################################################################
## line from [0,-1] to [0,-0.1] 
#path_specs = np.array([[0,-1,0,np.inf,0.9]])
## Line from [-1,3] to [3,3]:
#path_specs = np.array([[-1,3,-np.pi/2,np.inf,5]])
## Line from [-1,-1] to [-1,3]:
#path_specs = np.array([[-1,-1,0,np.inf,4]])


###############################################################################
## ALTERNATIVELY: get path specs from an SVG file: 
#   uses "parse_svg_for_path_following", a program that reads SVG images and converts the lines to waypoints. 
# NOTE: The "get_param" line is used to get the full path to the SVG file 
#   This is only necessary to get around a problem with locating files (unknown underlying reason). 
#   The Parameter is set in the Launch file! 
###############################################################################
#import mobrob.parse_svg_for_path_following as parsesvg    # This is a program that sorts out SVG files to find their waypoints. 
#path_file_svg = rospy.get_param('/path_file_svg')    # Get the parameter that has the file's full path
#path_specs = parsesvg.convert_svg_to_path_specs(path_file_svg, xlength=1., ylength=1.)    # Parse the SVG file for "d=" lines (paths)


####     CODE HERE: 
## Create other Paths of your own design. ##
####    CODE END



###############################################################################
# FIX THE PATH_SPECS to drive a line from the initial position to 
# the path starting point, before beginning the specified path 
# (if not already there): 
if not all( path_specs[0,0:2] == robot.r_center_world) :
    path_specs = np.append(np.array([robot.specify_line(robot.r_center_world[0], robot.r_center_world[1],path_specs[0,0],path_specs[0,1])]),path_specs,axis=0)
###############################################################################


########
# Here the actual SetPathToFollow node class.
########
    
class SetPathToFollow(Node): 
    def __init__(self):
        super().__init__('set_path_to_follow')
        
        # Action Client for the "ME439FollowPath" Action in "closed_loop_path_follower.py".
        self.CL_action_client = ActionClient(self, ME439FollowPath, 'follow_path')  ## Name and Type have to match the Action Server. 
        self.status = GoalStatus.STATUS_EXECUTING
        
        # Set a "fraction_complete" field to keep track of how complete the path segment is. 
        self.fraction_complete = 0.
        
         
    # This is the member function that packs a Goal and sends it. 
    def send_path(self, path_spec):
        self.status = GoalStatus.STATUS_EXECUTING

        path_segment_spec = ME439FollowPath.Goal()
        path_segment_spec.x0 = float(path_spec[0])
        path_segment_spec.y0 = float(path_spec[1])
        path_segment_spec.theta0 = float(path_spec[2])
        path_segment_spec.radius = float(path_spec[3])
        path_segment_spec.length = float(path_spec[4])    
        
        # Wait for a server to be ready
        self.CL_action_client.wait_for_server()
        # Send a path segment spec
        send_goal_future = self.CL_action_client.send_goal_async(path_segment_spec, feedback_callback=self.feedback_callback)
        # Add a callback to receive the response on whether the goal was accepted or not. 
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        # reset "fraction_complete" asset 
        self.fraction_complete = 0
        
        return send_goal_future
        
    def goal_response_callback(self, response_future):
        goal_response_handle = response_future.result()
        self.get_logger().info('Goal Response: {}'.format(goal_response_handle.accepted))
        if not goal_response_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            rclpy.shutdown()
            
        # If it gets here, the Goal was accepted
        self.get_logger().info('Goal accepted :)')
        # print(goal_response_handle)
        
        # Therefore request a result (to arrive sometime in the future)...
        get_result_future = goal_response_handle.get_result_async()
        # and add a Result callback to receive it when it arrives. 
        get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, action_result_future):
        # A confusing line that uses "result" too many times, each meaning something different, as part of unpacking the Action's Result. 
        result = action_result_future.result().result
        self.fraction_complete = result.fraction_complete
        self.status = GoalStatus.STATUS_SUCCEEDED        # Here the Result should be a "fraction_complete" variable. 
        self.get_logger().info('Result: {0}'.format(result.fraction_complete))
    
    # A more straightforward callback to report the Fraction Complete (continuing feedback from the Action):
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.fraction_complete = feedback.fraction_complete
        self.get_logger().info('Received feedback: Fraction Complete: {0}'.format(feedback.fraction_complete))
        

## Here there is a bit more logic: 
def main(args=None):
    rclpy.init(args=args)
    set_path_to_follow_instance = SetPathToFollow()
    
    # Extra logic to step through "path_specs" that were created above. 
    for ii, path_spec in zip(range(len(path_specs)), path_specs):
        
        ### This one works! With rclpy.init and creation of the Instance above the For loop. 
        future = set_path_to_follow_instance.send_path(path_spec)  # Actually send the goal. 
        set_path_to_follow_instance.get_logger().info('segment {}'.format(ii))
        set_path_to_follow_instance.get_logger().info('path spec: {}'.format(path_spec))
        rclpy.spin_until_future_complete(set_path_to_follow_instance, future)  # Wait (spin) here until the "send_path" acceptance comes back complete. 
        # At this point the future should be complete (new path request is accepted or not), and we should wait for it to come back Accepted.  
        while set_path_to_follow_instance.status != GoalStatus.STATUS_SUCCEEDED:
            rclpy.spin_once(set_path_to_follow_instance)
        # ### Example of breaking this by exiting early. 
        # while set_path_to_follow_instance.fraction_complete < 0.4:
            # rclpy.spin_once(set_path_to_follow_instance)
    
    # When the for loop is complete, destroy the node. 
    set_path_to_follow_instance.destroy_node()    
    print('Specified Path Complete')


    

if __name__ == '__main__':
    main() 
