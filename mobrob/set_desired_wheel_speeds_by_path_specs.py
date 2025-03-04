#!/usr/bin/env python3

# =============================================================================
# Peter G. Adamczyk 
# 2018-10
# Updated 2024-10-08
# =============================================================================

import rclpy
from rclpy.node import Node
import traceback 
import numpy as np
# IMPORT the custom messages:(here "ME439WheelSpeeds")
from mobrob_interfaces.msg import ME439WheelSpeeds


# =============================================================================
#     Set up a time course of commands
# =============================================================================

# =============================================================================
# # NEW: Determine paths by lines, arcs, pivots and pauses, and create a 
# #  "robot" object to plan it for you. 
# =============================================================================

# Constant Parameters
wheel_width = 0.151 #rospy.get_param('/wheel_width_actual')
body_length = 0.235 #rospy.get_param('/body_length')
wheel_diameter = 0.060 #rospy.get_param('/wheel_diameter_actual')
wheel_radius = wheel_diameter/2.0


####    CODE HERE:
# Create a mobile robot object from the Imported module "me439_mobile_robot_class"
# REMEMBER to call the right file (i.e., use the _HWK if needed)
import mobrob.me439_mobile_robot_class_v01 as me439rbt
robot = me439rbt.robot(wheel_width, body_length, wheel_radius)

####    CODE HERE:
# Specify stage_settings as a collections of lines, arcs, pivots and pauses
# Use the functions in your Class (imported above as "me439rbt")
# Example: Move Forward and Back, 0.3 meters per second:  
# ** NOTE the signs on the backward driving: backward speed and backward distance! 
stage_settings = np.array( [ robot.plan_pause(1.0), robot.plan_line(0.3, 1.5), robot.plan_line(-0.3, -1.5), robot.plan_pause(2.0)] )
# Example: pause, forward, pause, pivot right 180 deg, pause, return to home, pause, turn, pause. 
# ** NOTE the signs on the Omega and Angle for the "plan_pivot" calls! 
stage_settings = np.array( [ robot.plan_pause(1.0), robot.plan_line(0.1, 0.3), robot.plan_pause(1.0), robot.plan_pivot(-1.0, -np.pi), robot.plan_pause(1.0), robot.plan_line(0.1, 0.3), robot.plan_pause(1.0), robot.plan_pivot(1.0, np.pi), robot.plan_pause(1.0)] )
####    CODE HERE: ADD YOUR OWN  


# Convert it into a numpy array
stage_settings_array = np.array(stage_settings)
# Convert the first column to a series of times (elapsed from the beginning) at which to switch settings. 
stage_settings_array[:,0] = np.cumsum(stage_settings_array[:,0],0)  # cumsum = "cumulative sum". The last Zero indicates that it should be summed along the first dimension (down a column). 

print('Stage Settings Array:')
print(stage_settings_array)
print('Total Stages: {} '.format(len(stage_settings_array)))
print('Stage Progress:') 
# =============================================================================
# # END of new section on planning with lines, arcs, pivots and pauses
# =============================================================================


class SetDesiredWheelSpeedsByPathSpecsNode(Node):
    def __init__(self):
        super().__init__('set_desired_wheel_speeds_by_path_specs')
        
        self.pub_speeds = self.create_publisher(ME439WheelSpeeds, '/wheel_speeds_desired', 1)
        self.stage = 0
        self.msg_out = ME439WheelSpeeds()
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_command_callback)
        self.tstart = self.get_clock().now()
        print(self.stage)

    def timer_command_callback(self): 
        try:
            future_stages = np.argwhere( stage_settings_array[:,0] >= (self.get_clock().now()-self.tstart).nanoseconds*1e-9 ) 
            if len(future_stages)>0:
                newstage = int(future_stages[0])
                if not newstage==self.stage:
                    self.stage = newstage
                    print(self.stage)
                    self.msg_out.v0 = float(stage_settings_array[self.stage,1])
                    self.msg_out.v1 = float(stage_settings_array[self.stage,2])
                    # Actually publish the message
                    self.pub_speeds.publish(self.msg_out)
                    # Log the info (optional)    
            else: 
                print('All Stages Complete')
                raise SystemExit
        
        except :
            # When done or Errored, Zero the speeds
            self.msg_out.v0 = 0.0
            self.msg_out.v1 = 0.0
            self.pub_speeds.publish(self.msg_out)
            raise 
        
        
def main(args=None):
    try: 
        rclpy.init(args=args)
        print('set_desired_speeds beginning')        
        set_speeds_by_specs_instance = SetDesiredWheelSpeedsByPathSpecsNode()
        rclpy.spin(set_speeds_by_specs_instance)
    except SystemExit:
        pass
    except: 
        traceback.print_exc()
        
    # Gets here only during shutdown. 
    set_speeds_by_specs_instance.destroy_node()
    rclpy.shutdown()
    
    print('set_desired_speeds exiting')

if __name__ == '__main__':
    main()
