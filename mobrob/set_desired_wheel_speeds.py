#!/usr/bin/env python3

# =============================================================================
# Peter G. Adamczyk 
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
    
# Use a new structure: 
# structure: 
# np.array([[duration, left_wheel_speed, right_wheel_speed], [duration, left_wheel_speed, right_wheel_speed], ...]
# 
# Example: Move Forward and Back, 2s each, 0.3 meters per second: 
stage_settings = np.array( [ [0.0, 0.0, 0.0], [5.0,0.3, 0.3], [5.0, -0.3, -0.3], [2.0, 0.0, 0.0]] )
# Example: forward, turn, return to home, turn. 
stage_settings = np.array( [ [0,0,0],[3,0.100,0.100],[1,0,0],[1.5,0.1592,-0.1592],[1,0,0],[3,0.100,0.100],[1,0,0],[1.5,-0.1592,0.1592],[1,0,0]] )

# Convert it into a numpy array
stage_settings_array = np.array(stage_settings)
# Convert the first column to a series of times (elapsed from the beginning) at which to switch settings. 
stage_settings_array[:,0] = np.cumsum(stage_settings_array[:,0],0)  # cumsum = "cumulative sum". The last Zero indicates that it should be summed along the first dimension (down a column). 

print('Stage Settings Array:')
print(stage_settings_array)
print('Total Stages: {} '.format(len(stage_settings_array)))
print('Stage Progress:') 

class SetDesiredWheelSpeedsNode(Node):
    def __init__(self):
        super().__init__('set_desired_wheel_speeds')
        
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
        set_speeds_instance = SetDesiredWheelSpeedsNode()
        rclpy.spin(set_speeds_instance)
    except SystemExit:
        pass
    except: 
        traceback.print_exc()
        
    # Gets here only during shutdown. 
    set_speeds_instance.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


