#!/usr/bin/env python3

# =============================================================================
# Peter G. Adamczyk 
# 2018-10-11
# Updated 2024-10-09
# =============================================================================

import matplotlib.pyplot as plt
import numpy as np 
import rclpy
from rclpy.node import Node
import traceback 
import mobrob.me439_mobile_robot_class_v00 as me439rbt
from geometry_msgs.msg import Pose2D

class MobileRobotAnimator(Node): 
    def __init__(self):
        super().__init__('mobile_robot_animator')
        self.f = 2.0  # Hz update rate
       
        # Constant Parameters
        wheel_width = 0.151 #rospy.get_param('/wheel_width_actual')
        body_length = 0.235 #rospy.get_param('/body_length')
        wheel_diameter = 0.060 #rospy.get_param('/wheel_diameter_actual')
        wheel_radius = wheel_diameter/2.0
        
        #Create a mobile robot object from the Imported module "me439_mobile_robot_class"
        self.robot = me439rbt.robot(wheel_width, body_length, wheel_radius)
    
        # Here start a Subscriber to the "robot_pose_simulated" topic.
        self.sub_robot_pose = self.create_subscription(Pose2D, '/robot_pose_simulated', self.set_robot_pose, 10)
        
        # # Call the animation function. This has a loop in it, so it won't exit.
        self.draw_robot_and_path()
        
        # Start a timer to drive updates of the figure using the "update_drawing" attribute. 
        self.timer = self.create_timer(1.0/self.f,self.update_drawing)
    
    # Callback to set the pose from incoming message. 
    def set_robot_pose(self, msg_in):
        self.robot.r_center_world = np.array([msg_in.x, msg_in.y])
        self.robot.theta= msg_in.theta
        self.robot.append_current_position_to_history()

    # Set up the plot
    def draw_robot_and_path(self):
        # Set up the Animation plot
        self.fig1= plt.figure()
        
        self.robotoutline, = plt.plot([], [], 'r-')
        self.robotpath, = plt.plot([],[], 'b--')
        plt.axis('equal')   # Note for some reason this has to come before the "xlim" and "ylim" or an explicit axis limits command "plt.axis([xmin,xmax,ymin,ymax])"
        plt.xlim(-1, 1)
        plt.ylim(-1, 1)
        plt.xlabel('x')
        plt.ylabel('y')
        plt.title(self.get_name())  # Title the plot with the name of the node, obtained from ROS.


    # Update the plot with new data - used as a timer callback
    def update_drawing(self):
        # If the figure has been closed, exit. 
        if not plt.fignum_exists(self.fig1.number):
            raise SystemExit
        
        current_outline = self.robot.get_outline_world_xy()
        past_path = self.robot.position_history_world   
        
        self.robotoutline.set_data(current_outline[0,:],current_outline[1,:])
        self.robotpath.set_data(past_path[:,0], past_path[:,1])
        
        ## Find limits of the plot and expand them to make sure the robot is on screen
        axlims = plt.axis()
        xmin = axlims[0]
        xmax = axlims[1]
        xwidth = xmax-xmin
        ymin = axlims[2]
        ymax = axlims[3]
        ywidth = ymax-ymin
        current_position = self.robot.r_center_world
        # Check robot position and expand the frame if necessary
        if (current_position[0] - xmin)/xwidth < 0.1:
            xmin = xmax - 1.5*xwidth
        if (current_position[0] - xmin)/xwidth > 0.9:
            xmax = xmin + 1.5*xwidth
        if (current_position[1] - ymin)/ywidth < 0.1:
            ymin = ymax - 1.5*ywidth
        if (current_position[1] - ymin)/ywidth > 0.9:
            ymax = ymin + 1.5*ywidth
        plt.axis([xmin,xmax,ymin,ymax])
        # This is the magic that actually displays the figure. 
        plt.pause(0.01)  
    
    
    
def main(args=None):
    try: 
        rclpy.init(args=args)
        print('robot_animator started')
        robot_animator_instance = MobileRobotAnimator()
        rclpy.spin(robot_animator_instance)
    except SystemExit: 
        print('Figure was closed')
    except: 
        traceback.print_exc()
    print('robot_animator exiting')
    
if __name__ == '__main__':
    main()