'''
Created on Nov 9, 2019

@author: duolu

    (1) keyboard-to-controller
    (2) joystick-to-controller
    (3) IMU
    (4) GPS
    (5) V2V sending
    (6) V2V receiving
    (7) V2V-to-controller
    (8) drive-by-wire




'''

import sys
import time
import socket
import struct
import threading

import numpy as np


# packages needed by the CAN bus
import can
import cantools

# packages needed by IMU over Arduino
import serial

# packages needed by GPS, search "sbp" with "swift" for details
import argparse
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer
from sbp.client.loggers.json_logger import JSONLogger
from sbp.navigation import SBP_MSG_BASELINE_NED
from sbp.navigation import MsgBaselineNED
from sbp.navigation import SBP_MSG_POS_LLH
from sbp.navigation import MsgPosLLH
from sbp.navigation import SBP_MSG_VEL_NED
from sbp.navigation import MsgVelNED
from sbp.orientation import SBP_MSG_BASELINE_HEADING

NR_VEHICLES = 3

# 1 for vehicle id, 1 for ts, 2 for control, 16 for IMU, 4 for GPS, in total 24
V2V_N = 24







class AGVPi(object):
    
    def __init__(self, vid, use_js):
        

        self.v_speed_dsr = 0
        self.steer_dsr = 0
        
        self.stop = False
        
        self.vid = vid
        self.use_js = use_js
        
        self.imu_ts = 0;
        self.imu_sample = np.zeros(16, dtype=np.float64)

        self.latitude = 0
        self.longitude = 0
        self.vn = 0
        self.ve = 0

        # these are the most recent V2V records received from other vehicles
        self.v2v_records = []
        
        # CAUTION: vehicle ID starts at 1 and here we just reserve the ID 0.
        # Hence, there will be NR_VEHICLES + 1 records, where NR_VEHICLES is
        # the actual number of vehicles with a positive ID such as 1, 2, etc.
        for i in range(NR_VEHICLES + 1):
            
            empty_record = np.zeros(V2V_N, np.float64)
            
            self.v2v_records.append(empty_record)


    def keyboard_thread(self):
        
        print('keyboard thread started.')
        
        while not self.stop:
            
            key = input('->')
            
            if key == 'exit':
                
                self.stop = True
        


    def process_js_event(self, ts, jsev_value, jsev_type, jsev_number):
        
        if jsev_type == 0x01:
            
            #print('Button event: ts=%u, number=%u, value=%d' % (ts, jsev_number, jsev_value))
            
            if jsev_number == 9:
                
                self.stop = True
            
            pass
        
        elif jsev_type == 0x02:
            
            #print('Axis event: ts=%u, number=%u, value=%d' % (ts, jsev_number, jsev_value))
            
            if jsev_number == 0:
                
                self.steer_dsr = -jsev_value * 150 // 32768
            
            if jsev_number == 1:
                
                self.v_speed_dsr = -jsev_value * 150 // 32768
            
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


    def imu_recv(self, sio, msg_len):
    
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
        
        imu_log_file = open('./log/imu_samples.csv', 'w')
        
        sio = serial.Serial('/dev/ttyACM0', 115200)
        
        time.sleep(1)
        
        state = 0
        msg_len = 0
        opcode = 0
        
        while not self.stop:
        
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
                    
                    ts, sample = self.imu_recv(sio, msg_len)
                    
                    self.imu_ts = time.time()
                    self.imu_sample = sample
                    
                    #print(self.imu_ts, self.imu_sample[0:5])
                    
                    sample_csv = ','.join(['%.5f' % value for value in sample])

                    imu_log_file.write(str(ts))
                    imu_log_file.write(',')
                    imu_log_file.write(sample_csv)
                    imu_log_file.write('\n')
                    imu_log_file.flush()
        
                else:
                    
                    print("Unknown opcode, just ignore.")
        
        
                state = 0
        
            else:
        
                print("Unknown state! Program is corrupted!")
            
            
            
        pass


    def gps_thread(self):
    
        print("testing GPS started.")
        
        log_file_pos = open('./log/gps_pos', 'w')
        log_file_vel = open('./log/gps_vel', 'w')
    
#         parser = argparse.ArgumentParser(description = "Swift Navigation SBP Example.")
#         parser.add_argument(
#             "-p",
#             "--port",
#             default=['/dev/ttyUSB0'],
#             nargs=1,
#             help="specify the serial port to use."
#         )
#         gps_args = parser.parse_args()
        
        with PySerialDriver('/dev/ttyUSB0', baud=115200) as gps_serial_driver:
            with Handler(Framer(gps_serial_driver.read, None, verbose=True)) as gps_handler:
    
    #     gps_serial_driver = PySerialDriver(args.port[0], baud=115200)
    #     
    #     gps_handler = Handler(Framer(gps_serial_driver.read, None, verbose=True))
        
                print("GPS opened.")
              
                while not self.stop:
            
                    for msg, metadata in gps_handler.filter(SBP_MSG_POS_LLH):
                        
                        self.latitude = msg.lat
                        self.longitude = msg.lon
                        
                        log_file_pos.write(str(msg))
                        log_file_pos.write('\n')
                        log_file_pos.flush()
                        
                        #print(msg, metadata)
                        break
                        
            
                
                    for msg, metadata in gps_handler.filter(SBP_MSG_VEL_NED):
                        current_gps_vel = msg
                        
                        #print("%.4f,%.4f,%.4f" % (msg.n * 1e-3, msg.e * 1e-3,
                        #                              msg.d * 1e-3))
            
                        log_file_vel.write(str(msg))
                        log_file_vel.write('\n')
                        log_file_vel.flush()
            
                        #print(msg, metadata)
                        break
                            
                
            
        
        pass


    def v2v_send_thread(self):
        
        print('v2v_send_thread started')
    
        
        #dst_ips = ["192.168.0.100", "192.168.0.11", "192.168.0.12", "192.168.0.13"]
        dst_ips = ["192.168.0.100", "192.168.137.1", "192.168.137.2", "192.168.137.3"]
        dst_port = 15000
        
        #src_ip = "192.168.0.1%d" % self.vid
        src_ip = "192.168.137.%d" % self.vid
        src_port = 15000 + self.vid
        
        print('This is %s' % src_ip)
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
        sock.bind((src_ip, src_port))
    
        #msg_array = np.random.rand(17)
    
        
        msg_array = np.zeros(V2V_N, dtype=np.float64)
    
    
        while not self.stop:
            
            for dst_ip in dst_ips:

                msg_array[0] = self.vid
                msg_array[1] = time.time()
                msg_array[2] = self.v_speed_dsr
                msg_array[3] = self.steer_dsr
                
                msg_array[4:20] = self.imu_sample
                msg_array[20] = self.latitude
                msg_array[21] = self.longitude
                msg_array[22] = self.vn
                msg_array[23] = self.ve
                
                
                # TODO: IMU and GPS
    
                sock.sendto(msg_array.tobytes(), (dst_ip, dst_port))
        
                #print('send to %s:%d' % (dst_ip, dst_port))
        
            time.sleep(0.1);
        
        pass
    
    
    
    
    def v2v_recv_thread(self):
        
        print('v2v_recv_thread started')
        
        v2v_log = open('./log/v2v_recv', 'w')
        
        port = 15000
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
       
        sock.bind(('', port))
        
        sock.settimeout(1)
        
        while not self.stop:
            
            try:
            
                data, addr = sock.recvfrom(1024)
            
            except:
                
                continue
            
            v2v_record = np.frombuffer(data, dtype=np.float64)
            
            sender_id = int(v2v_record[0])
            
            #print('recv from %d' % sender_id)
            
            if sender_id >= 0 and sender_id < NR_VEHICLES:
                
                self.v2v_records[sender_id] = v2v_record
            
            
            v2v_csv = ','.join(['%.5f' % value for value in v2v_record])
            
            v2v_log.write(v2v_csv)
            v2v_log.write('\n')
            
#             print("[%12.3f]<%s:%d>\t" % (time.time(), addr[0], addr[1]), end="")
#     
#             print("%d" % v2v_record[0], end="")
#             print("\t", end="")
#             for j in range(12):
#         
#                 print("%5.2f, " % v2v_record[j + 1], end="")
#         
#             print()

        
        pass
    

    def v2v_to_control_thread(self):
        
        while not self.stop:
            
            # CAUTION: vehicle with ID == 1 is always the leader.
            
            self.v_speed_dsr = self.v2v_records[1][2]
            self.steer_dsr = self.v2v_records[1][3]
            
            time.sleep(0.05);
            
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
            
            #print('%.3f' % time.time(), self.v_speed_dsr, self.steer_dsr)
            
            test_drive_msg = can.Message(arbitration_id = test_drive_msg_struct.frame_id, 
                                              data=test_drive_msg_body)
            
            can_bus.send(test_drive_msg)
            
            time.sleep(0.05)
        
        
        pass


    def run(self, use_sensor):

        threads = []

#         x_keyboard = threading.Thread(target=self.keyboard_thread)
#         threads.append(x_keyboard)
#         x_keyboard.start()
        
        
        if self.use_js:
            
            x_js = threading.Thread(target=self.js_thread)
            threads.append(x_js)
            x_js.start()
        
        else:
            
            x_v2v_c = threading.Thread(target=self.v2v_to_control_thread)
            threads.append(x_v2v_c)
            x_v2v_c.start()

        x_v2v_send = threading.Thread(target=self.v2v_send_thread)
        threads.append(x_v2v_send)
        x_v2v_send.start()
    
        x_v2v_recv = threading.Thread(target=self.v2v_recv_thread)
        threads.append(x_v2v_recv)
        x_v2v_recv.start()
        
        
        if use_sensor:
            
            x_imu = threading.Thread(target=self.imu_thread)
            threads.append(x_imu)
            x_imu.start()
        
            x_gps = threading.Thread(target=self.gps_thread)
            threads.append(x_gps)
            x_gps.start()
        
        
        x_drive = threading.Thread(target=self.drive_by_wire)
        threads.append(x_drive)
        x_drive.start()
        
        
        for x in threads:
            
            x.join()
        



if __name__ == '__main__':
    
    use_js = False
    
    if len(sys.argv) < 3:
        
        print("Usage: python3 test_drive_by_v2v vid [js|v2v]")
        exit(0)

    vid = int(sys.argv[1])

    method = sys.argv[2]
    
    if method == 'js':
    
        use_js = True
        print('use js')
        
    elif method == 'v2v':
        
        use_js = False
        print('use v2v')
        
    else:
        
        print("Usage: python3 test_drive_by_v2v vid [js|v2v]")
        exit(0)
    
    
    agv = AGVPi(vid=vid, use_js=use_js)
    
    agv.run(use_sensor=False)
    
    pass

























