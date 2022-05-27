'''
Created on Nov 9, 2019

@author: duolu
'''

import os
import time

# packages needed by the CAN bus, 
import can
import cantools



def test_can_recv():
    
    print('test_can_recv started.')
    
    log_file = open('./log/can_recv', 'w')
    
    #os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")
    can_bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
    can_dbc = cantools.database.load_file('./dbc/AGVPi5_Test.dbc')

    print('can init OK')

    #print(can_dbc.messages)
    
    while True:
        
        msgs = can_bus.recv()
        
        
        
        try:
            decoded_msg = can_dbc.decode_message(msgs.arbitration_id, msgs.data)
             
            #print('%.3f' % time.time(), decoded_msg)
            log_file.write(str(decoded_msg))
            log_file.write('\n')
            log_file.flush()
             
        except Exception as e:
            
            print('error', e)
            pass
    
        
        pass

    log_file.close()



if __name__ == '__main__':
    
    test_can_recv()
    pass