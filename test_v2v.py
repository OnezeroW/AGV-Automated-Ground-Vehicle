'''
Created on Nov 16, 2019

@author: duolu
'''

import sys
import time
import socket
import threading

import serial

import numpy as np

class AGVPi(object):
    
    def __init__(self, vid):
        
        self.vid = vid
        
        self.imu_ts = 0;
        self.imu_sample = np.zeros(16, dtype=np.float64)
        
        pass





    def recv_payload(self, sio, msg_len):
    
        payload = sio.read(msg_len)
    
        #print(payload)
    
        ts = int.from_bytes(payload[0:4], byteorder='little')
    
        sample = np.frombuffer(payload[4:msg_len], dtype=np.float32)
    
    #     print(ts)
    #     print("\t", end="")
    #     for j in range(12):
    # 
    #         print("%7.4f, " % sample[j], end="")
    # 
    #     print()
    
        return (ts, sample)
    
    
    
    def imu_thread(self):
        
        print('imu_thread started.')
        
        sio = serial.Serial('/dev/ttyACM0', 115200)
        
        time.sleep(1)
        
        state = 0
        msg_len = 0
        opcode = 0
        
        while True:
        
            s = sio.read(1)
        
            # cast the received byte to an integer    
            c = s[0]
        
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
                    
                    ts, sample = self.recv_payload(sio, msg_len)
                    
                    self.imu_ts = ts
                    self.imu_sample = sample
                    
                    #print(self.imu_ts, self.imu_sample[0:5])
        
                else:
                    
                    print("Unknown opcode, just ignore.")
        
        
                state = 0
        
            else:
        
                print("Unknown state! Program is corrupted!")
            
            
            
        pass




    def v2v_send_thread(self):
        
        print('v2v_send_thread started')
    
        
        #dst_ips = ["192.168.0.100", "192.168.0.11", "192.168.0.12", "192.168.0.13"]
        dst_ips = ["192.168.0.100", "192.168.137.1", "192.168.137.2", "192.168.137.3"]
        dst_port = 15000
        
        #src_ip = "192.168.0.1%d" % self.vid
        src_ip = "192.168.137.%d" % self.vid
        src_port = 15000 + self.vid
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
        sock.bind((src_ip, src_port))
    
        #msg_array = np.random.rand(17)
    
        msg_array = np.zeros(17, dtype=np.float64)
    
    
        while True:
            
            for dst_ip in dst_ips:


                # read global state and send.

                msg_array[0] = self.imu_ts
                msg_array[1:] = self.imu_sample
    
                sock.sendto(msg_array.tobytes(), (dst_ip, dst_port))
        
            time.sleep(1);
        
        pass
    
    
    
    
    def v2v_recv_thread(self):
        
        print('v2v_recv_thread started')
        
        port = 15000
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
       
        sock.bind(('', port))
        
        while True:
            
            data, addr = sock.recvfrom(1024)
            
            v2v_record = np.frombuffer(data, dtype=np.float64)
            
            print("[%12.3f]<%s:%d>\t" % (time.time(), addr[0], addr[1]), end="")
    
            print("%d" % v2v_record[0], end="")
            print("\t", end="")
            for j in range(12):
        
                print("%5.2f, " % v2v_record[j + 1], end="")
        
            print()

        
        pass
    
    
    




def test_v2v():
    
    if len(sys.argv) < 2:
        
        print("missing argument <vid>. Usage: python3 test_v2v.py <vid>")
        
        return
    
    
    
    vid = int(sys.argv[1])
    
    assert(vid == 1 or vid == 2 or vid == 3)
    
    agv = AGVPi(vid)
    
    x_imu = threading.Thread(target=agv.imu_thread)
    
    x_imu.start()
    
    x_v2v_send = threading.Thread(target=agv.v2v_send_thread)    
    x_v2v_recv = threading.Thread(target=agv.v2v_recv_thread)

    
    x_v2v_send.start()
    x_v2v_recv.start()


    x_imu.join()    
    x_v2v_send.join()
    x_v2v_recv.join()
    
    pass




if __name__ == '__main__':
    
    test_v2v()
    
    pass























