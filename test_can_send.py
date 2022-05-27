'''
Created on Nov 9, 2019

@author: duolu
'''

import os
import time

# packages needed by the CAN bus, 
import can
import cantools



def test_can_send():
    
    
    #os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")
    can_bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
    can_dbc = cantools.database.load_file('./dbc/AGVPi3_Test.dbc')
    
    for i in range(5000):
        
        print(i)
    
        test_drive_msg_struct = can_dbc.get_message_by_name('Test_Drive')
        test_drive_msg_body = test_drive_msg_struct.encode( \
            {'v_speed_dsr': i % 100, \
             'steer_dsr': (i + 10) % 100})
        
        test_drive_msg = can.Message(arbitration_id = test_drive_msg_struct.frame_id, 
                                          data=test_drive_msg_body)
        
        can_bus.send(test_drive_msg)
        
        time.sleep(0.1)
    
    
    pass




if __name__ == '__main__':
    
    test_can_send()
    
    
    pass