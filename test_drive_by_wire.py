'''
Created on Nov 9, 2019

@author: duolu
'''

import os
import time
import struct

# packages needed by the CAN bus, 
import can
import cantools


import threading


class AGVPi(object):
    
    def __init__(self):
        

        self.v_speed_dsr = 0
        self.steer_dsr = 0
        
        self.stop = False


    def process_js_event(self, ts, jsev_value, jsev_type, jsev_number):
        
        if jsev_type == 0x01:
            
            #print('Button event: ts=%u, number=%u, value=%d' % (ts, jsev_number, jsev_value))
            
            if jsev_number == 9:
                
                self.stop = True
            
            pass
        
        elif jsev_type == 0x02:
            
            #print('Axis event: ts=%u, number=%u, value=%d' % (ts, jsev_number, jsev_value))
            
            if jsev_number == 0:
                
                self.steer_dsr = jsev_value * 100 // 32768
            
            if jsev_number == 1:
                
                self.v_speed_dsr = -jsev_value * 20 // 32768
            
            pass
        
        
        
        pass
    
    
    
    
    
    def js_thread(self):
        
        print('js thread started.')
        
        fn = '/dev/input/js0'
        jsdev = open(fn, 'rb')
        
        while not self.stop:
            
            jsev = jsdev.read(8)
            
            ts, jsev_value, jsev_type, jsev_number = struct.unpack('IhBB', jsev)
            
            self.process_js_event(ts, jsev_value, jsev_type, jsev_number)
        
        
        
        pass

    def drive_by_wire(self):
        
        
        #os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")
        can_bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
        can_dbc = cantools.database.load_file('./dbc/AGVPi3_Test.dbc')
        
        while not self.stop:
            
        
            test_drive_msg_struct = can_dbc.get_message_by_name('Test_Drive')
            test_drive_msg_body = test_drive_msg_struct.encode( \
                {'v_speed_dsr': self.v_speed_dsr, \
                 'steer_dsr': self.steer_dsr})
            
            print(self.v_speed_dsr, self.steer_dsr)
            
            test_drive_msg = can.Message(arbitration_id = test_drive_msg_struct.frame_id, 
                                              data=test_drive_msg_body)
            
            can_bus.send(test_drive_msg)
            
            time.sleep(0.05)
        
        
        pass

    def run(self):
        
        x_js = threading.Thread(target=self.js_thread)
        x_js.start()
        
        self.x_js = x_js
        
        self.drive_by_wire()
        



if __name__ == '__main__':
    
    agv = AGVPi()
    
    agv.run()
    
    pass

























