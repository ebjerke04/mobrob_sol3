# -*- coding: utf-8 -*-
"""
Created on Mon Aug  5 12:08:00 2024

@author: pgadamczyk-- Converted by Copilot 2024-08-05
Updated 2024-10-07
"""

# Below is the ROS 2 node in Python.

import rclpy
from rclpy.node import Node

import numpy as np
import serial
import traceback 

from mobrob_interfaces.msg import ME439SensorsRaw


class SensorsRawNode(Node):
    def __init__(self):
        super().__init__('sensors_raw')
        self.publisher_ = self.create_publisher(ME439SensorsRaw, '/sensors_data_raw', 1)
        
        self.serial_data_port = self.declare_parameter('/serial_data_port','/dev/ttyUSB0').value
        self.serial_baud_rate = self.declare_parameter('/serial_baud_rate',57600).value
        
    def operate(self):
        
        pub_sensors_message = ME439SensorsRaw()
        pub_sensors_message.a0 = 0
        pub_sensors_message.a1 = 0
        pub_sensors_message.a2 = 0
        pub_sensors_message.a3 = 0
        pub_sensors_message.a4 = 0
        pub_sensors_message.a5 = 0
        pub_sensors_message.u0 = 0
        pub_sensors_message.u1 = 0
        pub_sensors_message.u2 = 0
        pub_sensors_message.di = 0
        pub_sensors_message.db = ''
        pub_sensors_message.t = self.get_clock().now().to_msg()

        
        # Data comes in on the Serial port. Set that up and start it. 
        #----------setup serial--------------
        ser = serial.Serial(self.serial_data_port)  #serial port to Alamode or Arduino
        ser.baudrate = self.serial_baud_rate     # 115200 or *57600*
        ser.bytesize = 8
        ser.parity = 'N'
        ser.stopbits = 1
        ser.timeout = 1 # one second time out. 
        # Flush any backlog and read a line to make sure the port input is current. 
        ser.flushInput()
        ser.readline()
        
        while True:
            # set all the "new data" variables to zero. 
            newe0 = 0
            newe1 = 0
            newa0 = 0
            newa1 = 0
            newa2 = 0
            newa3 = 0
            newa4 = 0
            newa5 = 0
            newa6 = 0
            newa7 = 0
            newu0 = 0
            newu1 = 0
            newu2 = 0
            newdi = 0
            newdb = 0
    #        new_data_packet = 0
            try: 
                # Here we read the serial port for a string that looks like "e0:123456", which is an Encoder0 reading. 
                # When we get a reading, update the associated motor command
                line = ser.readline().decode().strip() #blocking function, will wait until read entire line
    #            print(line)
                line = line.split(":")
                data_type = line[0]
                data_str = line[1]
    #            print(data_type)
    #            print(line)
    
                if data_type == 'A0':
                    pub_sensors_message.a0 = int(data_str)
                    newa0 = 1
    #                print(a0)
    #                print("A0 = {0}").format(a0)
                elif data_type == 'A1':
                    pub_sensors_message.a1 = int(data_str)
                    newa1 = 1
    #                print(a1)
                elif data_type == 'A2':
                    pub_sensors_message.a2 = int(data_str)
                    newa2 = 1
    #                print(a2)
                elif data_type == 'A3':
                    pub_sensors_message.a3 = int(data_str)
                    newa3 = 1
    #                print(a3)
                elif data_type == 'A4':
                    pub_sensors_message.a4 = int(data_str)
                    newa4 = 1
    #                print(a4)
                elif data_type == 'A5':
                    pub_sensors_message.a5 = int(data_str)
                    newa5 = 1
    #                print(a5)
                elif data_type == 'A6':
                    pub_sensors_message.a6 = int(data_str)
                    newa6 = 1
    #                print(a6)
                elif data_type == 'A7':
                    pub_sensors_message.a7 = int(data_str)
                    newa7 = 1
    #                print(a7)
                elif data_type == 'U0':
                    pub_sensors_message.u0 = int(data_str)
                    newu0 = 1
    #                print(u0)
                elif data_type == 'U1':
                    pub_sensors_message.u1 = int(data_str)
                    newu1 = 1
    #                print(u1)
                elif data_type == 'U2':
                    pub_sensors_message.u2 = int(data_str)
                    newu2 = 1
    #                print(u2)
                elif data_type == 'DI':
                    pub_sensors_message.di = int(data_str)
                    newdi = 1
    #                print(di)
                elif data_type == 'DB':
                    pub_sensors_message.db = data_str # This one is special because it keeps the string of 0s and 1s. 
                    newdb = 1
    #                print(db)
                
                if newdb:
                    # Publish the sensor data
                    pub_sensors_message.t = self.get_clock().now().to_msg()
                    self.publisher_.publish(pub_sensors_message)
                    # self.get_logger().info('Published sensor data')
            
            except Exception:
                print('Bad line received on Arduino port - ignoring and continuing.')
                pass



def main(args=None):
    try: 
        rclpy.init(args=args)
        print('sensors_raw beginning')
        sensors_raw_instance = SensorsRawNode()
        sensors_raw_instance.operate()
    except KeyboardInterrupt:
        print('Keyboard Interrupt')
    except: 
        traceback.print_exc()
        
    sensors_raw_instance.destroy_node()
    rclpy.shutdown()
    
    print('sensors_raw exiting')


if __name__ == '__main__':
    main()

