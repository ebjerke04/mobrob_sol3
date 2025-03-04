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

from mobrob_interfaces.msg import ME439SensorsRaw, ME439SensorsProcessed


class SensorsProcessorNode(Node): 
    def __init__(self):
        super().__init__('sensors_processor')
        
        self.initializing = 1
        
        self.pub_sensors_proc = self.create_publisher(ME439SensorsProcessed, '/sensors_data_processed', 1)
        
        self.sub_sensors_raw = self.create_subscription(ME439SensorsRaw, '/sensors_data_raw', self.sensors_process, 1)
        

    def sensors_process(self,msg_in):
        speed_of_sound_meterspersec = 343.0
        try:     
            if self.initializing:
                self.t_previous = msg_in.t
                self.initializing = 0
            else: 
                msg_out = ME439SensorsProcessed()
                
                t = msg_in.t.sec+msg_in.t.nanosec*1e-9
                tprev = self.t_previous.sec+self.t_previous.nanosec*1e-9
                dt = ( t - tprev )
                msg_out.dt.sec = int(dt)
                msg_out.dt.nanosec = int((dt-msg_out.dt.sec)*1e9)
                
                msg_out.a0 = float(msg_in.a0)
                msg_out.a1 = float(msg_in.a1)
                msg_out.a2 = float(msg_in.a2)
                msg_out.a3 = float(msg_in.a3)
                msg_out.a4 = float(msg_in.a4)
                msg_out.a5 = float(msg_in.a5)
                msg_out.a6 = float(msg_in.a6)
                msg_out.a7 = float(msg_in.a7)
                
                msg_out.u0meters = msg_in.u0/10.0e6 * speed_of_sound_meterspersec
                msg_out.u1meters = msg_in.u1/10.0e6 * speed_of_sound_meterspersec
                msg_out.u2meters = msg_in.u2/10.0e6 * speed_of_sound_meterspersec

                di = msg_in.di
                msg_out.d2 = bool(di&(1<<2))
                msg_out.d3 = bool(di&(1<<3))
                msg_out.d4 = bool(di&(1<<4))
                msg_out.d5 = bool(di&(1<<5))
                msg_out.d6 = bool(di&(1<<6))
                msg_out.d7 = bool(di&(1<<7))
                msg_out.d8 = bool(di&(1<<8))
                msg_out.d9 = bool(di&(1<<9))
                msg_out.d10 = bool(di&(1<<10))
                msg_out.d11 = bool(di&(1<<11))
                msg_out.d12 = bool(di&(1<<12))
                msg_out.d13 = bool(di&(1<<13))
                 
                # Update "previous" values
                self.t_previous = msg_in.t
                
                # Publish the Processed message
                self.pub_sensors_proc.publish(msg_out)
       
                
        except Exception:
            traceback.print_exc()
            pass



def main(args=None):
    try: 
        rclpy.init(args=args)
        print('sensors_processor beginning')
        sensors_processor_instance = SensorsProcessorNode()
        rclpy.spin(sensors_processor_instance)
        
    except: 
        traceback.print_exc()
    
    print('sensors_processor ending')

if __name__ == '__main__':
    main()
