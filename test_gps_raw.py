'''
Created on Nov 10, 2019

@author: priori
'''

import time
import serial
import struct



def test_gps_raw_bytes():

    print("testing GPS raw data started.")
    
    log_file_raw = open('./log/gps_raw', 'w')
    
    sio = serial.Serial('/dev/ttyUSB0', 115200)
    
    while True:
    
        s = sio.read(1)
    
        # cast the received byte to an integer    
        c = s[0] & 0x000000FF
        
        log_file_raw.write('%02X ' % c)
        log_file_raw.flush()
        
        
    
    
    
    pass


def test_gps_raw_record():
    
    print("testing GPS raw record started.")
    
    log_file_raw_record = open('./log/gps_raw_record', 'w')
    
    sio = serial.Serial('/dev/ttyUSB0', 115200)
    
    while True:
    
        s = sio.read(1)
    
        # cast the received byte to an integer    
        c = s[0] & 0x000000FF


        if c == 0x55:
            
            header = sio.read(5)
            msg_type, sender, payload_len = struct.unpack('HHB', header)
    
            payload = sio.read(payload_len + 2)
            
            header_str = '0x%04X, 0x%04X, %d, ' % (msg_type, sender, payload_len)
            
            print(header_str)
            
            log_file_raw_record.write(header_str)
            log_file_raw_record.write(str(payload))
            log_file_raw_record.flush()
        
        else:
            
            continue
    
    pass




if __name__ == '__main__':
    
    
    #test_gps_raw_bytes()
    test_gps_raw_record()
    pass












