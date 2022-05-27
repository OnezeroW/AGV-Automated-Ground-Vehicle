'''
Created on Nov 8, 2019

@author: duolu
'''


import time
import serial

import numpy as np


def recv_payload(sio, msg_len):

    payload = sio.read(msg_len)

    #print(payload)

    ts = int.from_bytes(payload[0:4], byteorder='little')

    sample = np.frombuffer(payload[4:msg_len], dtype=np.float32)

    print(ts)
    print("\t", end="")
    for j in range(12):

        print("%7.4f, " % sample[j], end="")

    print()

    return (ts, sample)



def test_imu():
    
    print('test_imu started.')
    
    log_file = open('./log/imu_samples.csv', 'w')

    
    sio = serial.Serial('/dev/ttyACM0', 115200)
    #sio = serial.Serial('/dev/ttyUSB0', 115200)
    
    time.sleep(1)
    
    state = 0
    msg_len = 0
    opcode = 0
    
    print('Enter while loop! \n')
    counter = 0
    while True:
        #print(f'{counter} before sio.read() \n')
        s = sio.read(1)
        print(f'{s}, {counter} after sio.read() \n')
    
        # cast the received byte to an integer    
        c = s[0]
        if state != 0:
            print(f'c = {c}, state = {state}, counter = {counter} \n')
    
        if state == 0:
    
            if c == 68: # ASCII 68 is 'D'
                state = 1
            else:
                state = 0
    
        elif state == 1:
    
            if c == 76: # ASCII 76 is 'L'
                state = 2
            else:

                state = 0
    
        elif state == 2:
    
            state = 3
    
            msg_len = int(c)
    
    
        elif state == 3:
    
            opcode = int(c)
    
            if opcode == 0x85:
                print('recv_payload() \n')
                ts, sample = recv_payload(sio, msg_len)
                
                sample_csv = ','.join(['%.5f' % value for value in sample])
                
                log_file.write(str(ts))
                log_file.write(',')
                log_file.write(sample_csv)
                log_file.write('\n')
                log_file.flush()
    
            else:
                
                print("Unknown opcode, just ignore.")
    
    
            state = 0
    
        else:
    
            print("Unknown state! Program is corrupted!")
        
        counter += 1
        
    pass







if __name__ == '__main__':
    
    test_imu()
    

















