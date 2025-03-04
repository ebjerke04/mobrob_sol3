#!/usr/bin/env python3

# =============================================================================
# Peter G. Adamczyk 
# 2018-10-11
# Updated 2024-10-09
# =============================================================================

import rclpy
from rclpy.node import Node
import traceback 
import mobrob.me439_mobile_robot_class_v00 as me439rbt
from geometry_msgs.msg import Pose2D
from mobrob_interfaces.msg import ME439WheelSpeeds, ME439WheelAngles, ME439WheelDisplacements


class MobileRobotSimulator(Node): 
    def __init__(self):
        super().__init__('mobile_robot_simulator')
        self.t_previous = self.get_clock().now()
        self.f = 100.    # Simulation rate of the simulator node
        self.ndecimate = 10     # Publication rate of the Pose
        
        # Constant Parameters
        wheel_width = 0.151 #rospy.get_param('/wheel_width_actual')
        body_length = 0.235 #rospy.get_param('/body_length')
        wheel_diameter = 0.060 #rospy.get_param('/wheel_diameter_actual')
        wheel_radius = wheel_diameter/2.0
        
        #Create a mobile robot object from the Imported module "me439_mobile_robot_class"
        self.robot = me439rbt.robot(wheel_width, body_length, wheel_radius)
    
        # Here start a Subscriber to the "wheel_speeds_desired" topic.
        #   NOTE the Callback to the set_wheel_speeds function of the robot class.
        self.sub_wheel_speeds = self.create_subscription(ME439WheelSpeeds, '/wheel_speeds_desired', self.set_wheel_speed_targets, 10)  
    
        # Here a series of Publishers
        self.pub_robot_pose = self.create_publisher(Pose2D, '/robot_pose_simulated', 10)
        self.robot_pose_message = Pose2D()
    
        self.pub_robot_wheel_angles = self.create_publisher(ME439WheelAngles, '/wheel_angles_simulated', 10)
        self.robot_wheel_angles_message = ME439WheelAngles()
    
        self.pub_robot_wheel_displacements = self.create_publisher(ME439WheelDisplacements, '/wheel_displacements_simulated', 10)
        self.robot_wheel_displacements_message = ME439WheelDisplacements()
    
        # Timer object to set a simulation rate
        self.timer = self.create_timer(1.0/self.f, self.sim_tick)
        
        # and a tick counter
        self.count = 0
    
# =============================================================================
#     # Loop to run the simulation
# =============================================================================
    def sim_tick(self):
        self.t_current = self.get_clock().now()
        dt = (self.t_current - self.t_previous).nanoseconds*1e-9
        self.t_previous = self.t_current # save the time for the next tick.
        self.robot.integration_step(dt)
            
        self.count += 1
        
        # Only publish every "ndecimate" ticks
        if not self.count % self.ndecimate:  # (same as: "if count%ndecimate == 0")
            # when it gets here (every ndecimate-th simulation step) we want to actually publish the data
            # # Maybe log the current position?
            # robot.append_current_position_to_history()
        
            # Now publish the pose
            self.robot_pose_message.x = self.robot.r_center_world[0]
            self.robot_pose_message.y = self.robot.r_center_world[1]
            self.robot_pose_message.theta = self.robot.theta
            self.pub_robot_pose.publish(self.robot_pose_message)
            
            # And the encoder angles
            self.robot_wheel_angles_message.ang0 = self.robot.left_wheel_angle
            self.robot_wheel_angles_message.ang1 = self.robot.right_wheel_angle
            self.pub_robot_wheel_angles.publish(self.robot_wheel_angles_message)
            
            # And the wheel displacements
            self.robot_wheel_displacements_message.disp0 = self.robot.left_wheel_distance_traveled
            self.robot_wheel_displacements_message.disp1 = self.robot.right_wheel_distance_traveled
            self.pub_robot_wheel_displacements.publish(self.robot_wheel_displacements_message)
                
        
    # Callback function to set wheel speeds in the robot object
    def set_wheel_speed_targets(self, msg_in): 
        # # Optional: do a partial simulation step here to account for fractional simulation time with the old wheel speeds! 
        # t_current = self.get_clock().now()
        # dt = (t_current - self.t_previous).to_sec()
        # self.t_previous = t_current      # save the current time as the previous time, for the next use. 
        # self.robot.integration_step(dt)
        self.robot.set_wheel_speeds(msg_in.v0, msg_in.v1)
    
    
    
def main(args=None):
    try: 
        rclpy.init(args=args)
        simulator_instance = MobileRobotSimulator()
        rclpy.spin(simulator_instance)
    except: 
        traceback.print_exc()
    
    
    
if __name__ == '__main__':
    main()
