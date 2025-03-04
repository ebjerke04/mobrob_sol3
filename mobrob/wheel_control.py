#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
import serial
import traceback 

# Use GPIOZERO for both encoders and motors
from gpiozero import RotaryEncoder, PhaseEnableMotor

# IMPORT the custom messages: 
# we import it from the ROS package we created it in (here "mobrob_interfaces") with an extension of .msg ...
# and actually import the message type by name (here "ME439SensorsRaw" and others)
from mobrob_interfaces.msg import ME439SensorsRaw, ME439WheelSpeeds, ME439MotorCommands, ME439WheelAngles, ME439WheelDisplacements

# IMPORT the "encoders_and_motors" module so we can call its pieces as functions. 
import mobrob.encoders_and_motors as encmot


class WheelControlNode(Node): 
    def __init__(self): 
        super().__init__('wheel_control')
        
        # Subscribe to "wheel_speeds_desired" to get values for the controller. 
        sub_wheel_speeds = self.create_subscription(ME439WheelSpeeds, '/wheel_speeds_desired', self.set_wheel_speed_targets, 1)
        #   NOTE the Callback to the "self.set_wheel_speed_targets" method, which will update the setting of a motor controller that is called from the main loop below. 
        
        # Encoders and Motors from GPIOZERO
        self.e0 = RotaryEncoder(17,27,max_steps=0,bounce_time=None)
        self.e1 = RotaryEncoder(23,24,max_steps=0,bounce_time=None)
        self.m0 = PhaseEnableMotor(5,12)
        self.m1 = PhaseEnableMotor(6,13)
        
        # This Publish Wheel Displacements for use in Dead Reckoning
        self.pub_motor_commands = self.create_publisher(ME439MotorCommands, '/motor_commands', 1)
        self.pub_wheel_displacements = self.create_publisher(ME439WheelDisplacements, '/wheel_displacements', 1)
        ### Consider also ME439MotorCommands and ME439WheelAngles

        # Attributes to keep track of time rate and times through the loop
        self.t_previous = self.get_clock().now()
        self.publishCount=0
        
        # Parameters
        self.counts_per_encoder_revolution = self.declare_parameter('/counts_per_encoder_revolution', 3.0).value
        self.gear_ratio = self.declare_parameter('/gear_ratio', 120).value
        self.wheel_radius = self.declare_parameter('/wheel_radius_model', 0.030).value 
        
        self.encoder_update_rate_hz = self.declare_parameter('/encoder_update_rate_hz', 100).value 
        
        self.e0_direction_sign = self.declare_parameter('/left_encoder_sign', 1).value 
        self.e1_direction_sign = self.declare_parameter('/right_encoder_sign', -1).value 
        
        self.m0_direction_sign = self.declare_parameter('/left_motor_sign', 1).value 
        self.m1_direction_sign = self.declare_parameter('/right_motor_sign', -1).value 
        
        self.integral_error_max = self.declare_parameter('/vel_integral_limit', 0.2).value 
        self.integral_resetting = self.declare_parameter('/integral_resetting', False).value 
        self.cmd_rate_of_change_max = self.declare_parameter('/cmd_rate_of_change_max', 2).value 
        self.motor_command_max = self.declare_parameter('/motor_command_max', 1.0).value 
        
        self.Kf0 = self.declare_parameter('/vel_left_f', 0.).value 
        self.Kp0 = self.declare_parameter('/vel_left_p', 2.).value 
        self.Ki0 = self.declare_parameter('/vel_left_i', 20.).value 
        self.Kd0 = self.declare_parameter('/vel_left_d', 0.01).value 
        self.Kf1 = self.declare_parameter('/vel_right_f', 0.).value 
        self.Kp1 = self.declare_parameter('/vel_right_p', 2.).value 
        self.Ki1 = self.declare_parameter('/vel_right_i', 20.).value 
        self.Kd1 = self.declare_parameter('/vel_right_d', 0.01).value 
        
        # Max encoder increment (full speed) - useful for eliminating errors
        self.max_enc_incr = self.declare_parameter('/max_enc_incr', 100).value 


        #==============================================================================
        #     # Create two Quadrature Encoder instances, one for each wheel 
        #==============================================================================
        #   constructor function call: new_variable = encmot.quadrature_encoder(serial_string_identifier, counts_per_encoder_revolution, gear_ratio, wheel_radius, forward_encoder_rotation_sign)
        self.qe0 = encmot.quadrature_encoder("E0", self.counts_per_encoder_revolution,self.gear_ratio, self.wheel_radius, self.e0_direction_sign)
        self.qe1 = encmot.quadrature_encoder("E1",self.counts_per_encoder_revolution,self.gear_ratio, self.wheel_radius, self.e1_direction_sign)
        
        #==============================================================================
        #     # Create two motor controller instances, one for each wheel
        #==============================================================================
        #   constructor function call: FPID_controller (): new_object = encmot.FPID_controller(motor,Kf, Kp,Ki,Kd, error_integral_limit=np.inf, integral_resetting = True, motor_command_max_rate_of_change=1500., forward_motor_command_sign=1)
        self.mc0 = encmot.FPID_controller(motor=self.m0, Kf=self.Kf0, Kp=self.Kp0, Ki=self.Ki0, Kd=self.Kd0, error_integral_limit = self.integral_error_max, integral_resetting = self.integral_resetting, motor_command_max_rate_of_change = self.cmd_rate_of_change_max, motor_command_max = self.motor_command_max, forward_motor_command_sign = self.m0_direction_sign)
        self.mc1 = encmot.FPID_controller(motor=self.m1, Kf=self.Kf1, Kp=self.Kp1, Ki=self.Ki1, Kd=self.Kd1, error_integral_limit = self.integral_error_max, integral_resetting = self.integral_resetting, motor_command_max_rate_of_change = self.cmd_rate_of_change_max, motor_command_max = self.motor_command_max, forward_motor_command_sign = self.m1_direction_sign)

        self.update_encoders()
        
        self.timer = self.create_timer(1.0/self.encoder_update_rate_hz, self.update_encoders)

    #==============================================================================
    # # Function to send updated Commands directly to the motor controllers.
    #==============================================================================
    #   These commands come from the Subscriber above. 
    def set_wheel_speed_targets(self, msg_in):
        self.mc0.update_target_value(msg_in.v0)
        self.mc1.update_target_value(msg_in.v1)
        
    def update_encoders(self):
        #Make messages to fill later.
        msg_motor_commands = ME439MotorCommands()
        msg_motor_commands.cmd0 = 0
        msg_motor_commands.cmd1 = 0

        msg_wheel_angles = ME439WheelAngles()
        msg_wheel_angles.ang0 = 0. 
        msg_wheel_angles.ang1 = 0. 

        msg_wheel_displacements = ME439WheelDisplacements()
        msg_wheel_displacements.disp0 = 0.
        msg_wheel_displacements.disp1 = 0.
        
        #read the encoders:
        leftEnc = self.e0.steps 
        rightEnc = self.e1.steps 
        
        # get the time basis
        t = self.get_clock().now() 
        dt = (t-self.t_previous).nanoseconds*1e-9
        self.t_previous = t # store it for the next round

        #processing for left encoder #0.
        #update PID loops
        self.qe0.update(leftEnc, dt)
        self.mc0.update_current_value(self.qe0.meters_per_second, dt) 
        msg_motor_commands.cmd0 = self.mc0.motor_command
        msg_wheel_angles.ang0 = self.qe0.radians
        msg_wheel_displacements.disp0 = self.qe0.meters
        
        #processing for right encoder #1.
        #update PID loops
        self.qe1.update(rightEnc, dt)
        self.mc1.update_current_value(self.qe1.meters_per_second, dt) 
        msg_motor_commands.cmd1 = self.mc1.motor_command
        msg_wheel_angles.ang1 = self.qe1.radians
        msg_wheel_displacements.disp1 = self.qe1.meters
        
        self.publishCount = self.publishCount +1
        #only publish every 10 loops to avoid overloading ROS (should be ~10Hz)
        if (self.publishCount % 10 == 0):
            self.pub_motor_commands.publish(msg_motor_commands)
            # Publish a message to the "/robot_wheel_angles" topic
            # self.pub_wheel_angles.publish(msg_wheel_angles)
            # Publish a message to the "/robot_wheel_displacements" topic
            self.pub_wheel_displacements.publish(msg_wheel_displacements)



def main(args=None):
    try: 
        rclpy.init(args=args)
        wheel_control_node_instance = WheelControlNode()
        rclpy.spin(wheel_control_node_instance)
        
    except: 
        traceback.print_exc()
        
    # self.m0.stop()
    # self.m1.stop()

    # # Gets here only during shutdown. 
    # wheel_control_node_instance.destroy_node()
    # rclpy.shutdown()



if __name__ == '__main__':
    main()

