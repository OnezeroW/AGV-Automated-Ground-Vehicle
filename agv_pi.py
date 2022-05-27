'''

AGV management and v2v code running on the Raspberry Pi.


Created on Oct 1, 2019

@author: duolu

TODO:

    * start / stop logging
    * enable / disable
    
    * V2I send
    * Redo V2I message




'''

import os
import sys
import time
import socket
import struct
import threading

import json

import math
import numpy as np

# packages needed by the CAN bus, 
import can
import cantools

# packages needed by IMU over Arduino
import serial


NR_VEHICLES = 3 + 1

# 1 for vehicle id, 1 for ts, 2 for control, 16 for IMU, 4 for GPS, in total 24
V2V_N = 24

# 1 vid, 2 timestamps, 12 chassis states, 16 IMU states, 15 GPS states
V2I_N = 1 + 2 + 12 + 16 + 15




class AGVPi(object):
    '''
    This is the class represents a manager which is in charge of the vehicle.
    It manages all resources on the vehicle such as button, network, etc., 
    and it connects to the centrol control panel and other vehicles.
    
    Basically it starts multiple threads, where each of them runs an infinite 
    loop. 
    
    (1) The GPS thread queries the GPS module every one second and store it to
        the states hold in the class.
    (2) The CAN thread regularly reads states from the chassis and write sensor
        readings to the controller on CAN.
    (3) The Arduino thread read out data from the Arduino regularly (currently
        only IMU data).
    
    
    '''


    def __init__(self):
        
        # global initial configuration
        
        self.vid = 0
        self.mode = 'none'
        
        # load initial configuration from config file
        
        self.init_with_config_file()

        # global states

        self.stop = False
        
        
        self.js_speed_dsr = 0
        self.js_steer_dsr = 0
        
        #IMU states
        self.imu_ts = 0
        self.imu_sample = np.zeros(16, dtype=np.float64)

        # GPS states
        self.gps_pos_llh = (0,0,0,0,0,0,0)
        self.gps_baseline_ned = (0,0,0,0,0,0,0)
        self.gps_vel_ned = (0,0,0,0,0,0,0)
        


        # Chassis and vehicle controller states
        self.vehicle_speed_dsr = 0
        self.vehicle_steer_dsr = 0
        self.vehicle_state = np.zeros(26, dtype=np.float64)
        
        self.ctrl_state = np.zeros(20, dtype=np.float64)
        
        self.ctrl_command = np.zeros(10, dtype=np.float64)

        # V2V states

        # these are the most recent V2V records received from other vehicles
        # CAUTION: the actual transmitted message is in binary format (C struct),
        # not numpy array.
        self.v2v_records = np.zeros((NR_VEHICLES, V2V_N), np.float64)
        

        # V2I states

        # most recently V2I command (received) and state report (sent)
        # CAUTION: the actual transmitted message is in binary format (C struct),
        # not numpy array.
        self.v2i_command_msg = np.zeros(16, dtype=np.float64)
        self.v2i_vstate_msg = np.zeros(V2I_N, dtype=np.float64)

        
        pass


    def init_with_config_file(self):
        
        with open('./agvpi.conf',mode='r') as fd:
        
            config_str = fd.read()
        
            config = json.loads(config_str)
            
            self.vid = int(config['vid'])
            self.mode = config['mode']
            
            #print(self.vid, self.mode)
        
        pass


    def gps_log_record(self, fd, ts, tup):
        
        msg_csv = ','.join([str(value) for value in tup])
        
        fd.write('%f,' % ts)
        fd.write(msg_csv)
        fd.write('\n')
        fd.flush()
    

    def gps_thread(self):
        
        print("GPS thread started.")
        
        log_file_raw_record = open('./log/gps_raw_record', 'w')
        
        log_pos_llh = open('./log/gps_pos_llh.csv', 'w')
        log_baseline_ned = open('./log/gps_baseline_ned.csv', 'w')
        log_vel_ned = open('./log/gps_vel_ned.csv', 'w')
    
        sio = serial.Serial('/dev/ttyUSB0', 115200)
        
        while not self.stop:
        
            s = sio.read(1)
        
            # cast the received byte to an integer    
            c = s[0] & 0x000000FF
    
    
            if c == 0x55:
                
                ts = time.time()
                
                header = sio.read(5)
                msg_type, sender, payload_len = struct.unpack('HHB', header)
        
                payload = sio.read(payload_len + 2) 
                
                header_str = '0x%04X, 0x%04X, %d, %d' % (msg_type, sender, payload_len, len(payload))
    
                #print(header_str)
    
                if msg_type == 0x0102: # MSG_GPS_TIME
                    
                    gps_time = struct.unpack('<HIiBH', payload)
    
                elif msg_type == 0x0103: # MSG_UTC_TIME
                    
                    utc_time = struct.unpack('<BIHBBBBBIH', payload)
    
                elif msg_type == 0x0208: # MSG_DOPS
                    
                    dops = struct.unpack('<IHHHHHBH', payload)
                
                elif msg_type == 0x0209: # MSG_POS_ECEF
                    
                    pos_ecef = struct.unpack('<IdddHBBH', payload)
    
                elif msg_type == 0x0214: # MSG_POS_ECEF_COV
                    
                    pos_ecef_cov = struct.unpack('<IdddffffffBBH', payload)
                
                elif msg_type == 0x020A: # MSG_POS_LLH
                    
                    pos_llh = struct.unpack('<IdddHHBBH', payload)
                    self.gps_pos_llh = pos_llh
                    
                    self.gps_log_record(log_pos_llh, ts, pos_llh)
    
                elif msg_type == 0x0211: # MSG_POS_LLH_COV
                    
                    pos_llh_cov = struct.unpack('<IdddffffffBBH', payload)
    
                elif msg_type == 0x020B: # MSG_BASELINE_NED
                    
                    baseline_ecef = struct.unpack('<IiiiHBBH', payload)
    
                elif msg_type == 0x020C: # MSG_POS_LLH
                    
                    baseline_ned = struct.unpack('<IiiiHHBBH', payload)
                    self.gps_baseline_ned = baseline_ned
                    
                    self.gps_log_record(log_baseline_ned, ts, baseline_ned)
                
                elif msg_type == 0x020D: # MSG_VEL_ECEF
                    
                    vel_ned_ecef = struct.unpack('<IiiiHBBH', payload)
                    
                elif msg_type == 0x0215: # MSG_VEL_ECEF_COV
                    
                    vel_ned_ecef_cov = struct.unpack('<IiiiffffffBBH', payload)
    
                elif msg_type == 0x020E: # MSG_VEL_NED
                    
                    vel_ned = struct.unpack('<IiiiHHBBH', payload)
                    self.gps_vel_ned = vel_ned
                    
                    self.gps_log_record(log_vel_ned, ts, vel_ned)
                
                elif msg_type == 0x0212: # MSG_VEL_NED_COV
                    
                    vel_ned_cov = struct.unpack('<IiiiffffffBBH', payload)
    
                elif msg_type == 0x0213: # MSG_VEL_BODY
                    
                    vel_body = struct.unpack('<IiiiffffffBBH', payload)
    
                elif msg_type == 0x0210: # MSG_AGE_CORECTIONS    
                    
                    age_corrections = struct.unpack('<IHH', payload)
    
    
                else:
                        
                    # not interested in other messages
                    pass
                
                
                
                
                log_file_raw_record.write('%f, ' % ts)
                log_file_raw_record.write(header_str)
                log_file_raw_record.write(payload.hex())
                log_file_raw_record.write('\n')
                log_file_raw_record.flush()
            
            else:
                
                continue


    def arduino_sensor_recv(self, sio, msg_len):
    
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


    def arduino_sensor_thread(self):
        
        print('arduino_sensor_thread started.')
        
        imu_log_file = open('./log/imu_samples.csv', 'w')
        
        sio = serial.Serial('/dev/ttyACM0', 115200)
        
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
                    
                    imu_ts, sample = self.arduino_sensor_recv(sio, msg_len)
                    
                    ts = time.time()
                    self.imu_ts = imu_ts
                    self.imu_sample = sample
                    
                    #print(self.imu_ts, self.imu_sample[0:5])
                    
                    imu_csv = ','.join(['%f' % value for value in sample])

                    imu_log_file.write('%f, %f, ' % (ts, imu_ts))
                    imu_log_file.write(imu_csv)
                    imu_log_file.write('\n')
                    imu_log_file.flush()
        
                else:
                    
                    print("Unknown opcode, just ignore.")
        
        
                state = 0
        
            else:
        
                print("Unknown state! Program is corrupted!")


    def chassis_can_recv_thread(self):
        
        print('chassis_can_recv_thread started.')

        chassis_log_file = open('./log/chassis_can.csv', 'w')
        chassis_log_file_raw = open('./log/chassis_can.raw', 'w')

        ctrl_log_file = open('./log/ctrl_can.csv', 'w')


        can_bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
        can_dbc = cantools.database.load_file('./dbc/AGVPi5_Test.dbc')
        
        
        
        while not self.stop:
        
            # receive chassis message
            
            time.sleep(0.01)
            
            msg = can_bus.recv()
            
            
            try:
                dmsg = can_dbc.decode_message(msg.arbitration_id, msg.data)
            except:
                pass


            #print('%X' % msg.arbitration_id)
            
            id = int(msg.arbitration_id)

            if msg.arbitration_id == 0x0C000101:
                
                self.vehicle_state[0] = dmsg['Str_Front']
                self.vehicle_state[1] = dmsg['Str_Rear']
                self.vehicle_state[2] = dmsg['Tor_FL']

            if msg.arbitration_id == 0x0C000102:
                
                self.vehicle_state[3] = dmsg['Tor_FR']
                self.vehicle_state[4] = dmsg['Tor_RL']
                self.vehicle_state[5] = dmsg['Tor_RR']

            if msg.arbitration_id == 0x0B520101:
                
                self.vehicle_state[6] = dmsg['Pre_Stu_1']
                self.vehicle_state[7] = dmsg['Torque_Fbk_1']

            if msg.arbitration_id == 0x0B520201:
                
                self.vehicle_state[8] = dmsg['Motor_Spd_1']
                self.vehicle_state[9] = dmsg['Direction_1']
        
            if msg.arbitration_id == 0x0B520102:
                
                self.vehicle_state[10] = dmsg['Pre_Stu_2']
                self.vehicle_state[11] = dmsg['Torque_Fbk_2']

            if msg.arbitration_id == 0x0B520202:
                
                self.vehicle_state[12] = dmsg['Motor_Spd_2']
                self.vehicle_state[13] = dmsg['Direction_2']
        
            if msg.arbitration_id == 0x0B520103:
                
                self.vehicle_state[14] = dmsg['Pre_Stu_3']
                self.vehicle_state[15] = dmsg['Torque_Fbk_3']

            if msg.arbitration_id == 0x0B520203:
                
                self.vehicle_state[16] = dmsg['Motor_Spd_3']
                self.vehicle_state[17] = dmsg['Direction_3']
        
            if msg.arbitration_id == 0x0B520104:
                
                self.vehicle_state[18] = dmsg['Pre_Stu_4']
                self.vehicle_state[19] = dmsg['Torque_Fbk_4']

            if msg.arbitration_id == 0x0B520204:
                
                self.vehicle_state[20] = dmsg['Motor_Spd_4']
                self.vehicle_state[21] = dmsg['Direction_4']

                # This is the the last message of one iteration, now save the
                # vehicle states to the log file.
                
                chassis_csv = ','.join(['%f' % value for value in self.vehicle_state])
 
                ts = time.time()
                chassis_log_file.write('%f,' % ts)
                chassis_log_file.write(chassis_csv)
                chassis_log_file.write('\n')
                chassis_log_file.flush()

            if msg.arbitration_id == 0x00000381:
                
                self.vehicle_state[22] = dmsg['Direction_Str_1']
                self.vehicle_state[23] = dmsg['Position_Str_1']

            if msg.arbitration_id == 0x00000382:
                
                self.vehicle_state[24] = dmsg['Direction_Str_2']
                self.vehicle_state[25] = dmsg['Position_Str_2']


            # control 


            if msg.arbitration_id == 0x00000201:
                
                self.ctrl_state[0] = dmsg['Lati_h']
                self.ctrl_state[1] = dmsg['Lati_l']

            if msg.arbitration_id == 0x00000202:
                
                self.ctrl_state[2] = dmsg['Long_h']
                self.ctrl_state[3] = dmsg['Long_l']

            if msg.arbitration_id == 0x00000203:
                
                self.ctrl_state[4] = dmsg['GPS_VelN']

            if msg.arbitration_id == 0x00000204:
                
                self.ctrl_state[5] = dmsg['GPS_VelE']


            if msg.arbitration_id == 0x00000205:
                
                self.ctrl_state[8] = dmsg['V']
                self.ctrl_state[9] = dmsg['Vx']
                self.ctrl_state[10] = dmsg['Vy']

            if msg.arbitration_id == 0x00000206:
                
                self.ctrl_state[11] = dmsg['Heading']
                self.ctrl_state[12] = dmsg['Yaw']
                self.ctrl_state[13] = dmsg['Sideslip']

                ctrl_csv = ','.join(['%f' % value for value in self.ctrl_state])
                 
                ts = time.time()
                ctrl_log_file.write('%f,' % ts)
                ctrl_log_file.write(ctrl_csv)
                ctrl_log_file.write('\n')
                ctrl_log_file.flush()


            if msg.arbitration_id == 0x00000207:
                
                self.ctrl_state[14] = dmsg['ey']
                self.ctrl_state[15] = dmsg['eh']

            if msg.arbitration_id == 0x00000208:
                
                self.ctrl_state[16] = dmsg['r']
                self.ctrl_state[17] = dmsg['ay']

                

                
            
#             ts = time.time()
#             chassis_log_file_raw.write('%f,' % ts)
#             chassis_log_file_raw.write('%X, ' % msg.arbitration_id)
#             chassis_log_file_raw.write(msg.data.hex())
#             chassis_log_file_raw.write('\n')
#             chassis_log_file_raw.flush()
        
        pass


    def chassis_can_send_thread(self):
        
        print('chassis_can_send_thread started.')


        can_send_log_file = open('./log/pi_can_send.csv', 'w')
        
        #os.system("sudo /sbin/ip link set can0 up type can bitrate 500000")
        can_bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
        can_dbc = cantools.database.load_file('./dbc/AGVPi5_Test.dbc')
        
        while not self.stop:
            
            
            # drive by wire
        
            test_drive_msg_struct = can_dbc.get_message_by_name('Test_Drive')
            test_drive_msg_body = test_drive_msg_struct.encode( \
                {'v_speed_dsr': self.vehicle_speed_dsr, \
                 'steer_dsr': self.vehicle_steer_dsr})
            
            #print('%.3f' % time.time(), self.v_speed_dsr, self.steer_dsr)
            
            test_drive_msg = can.Message(arbitration_id = test_drive_msg_struct.frame_id, 
                                              data=test_drive_msg_body)
            
            can_bus.send(test_drive_msg)
            
            
            self.ctrl_command[0] = self.vehicle_speed_dsr
            self.ctrl_command[1] = self.vehicle_steer_dsr
            
            
            # GPS
            
            latitude = self.gps_pos_llh[1]
            longitude = self.gps_pos_llh[2]
            
            
            
            vn = int(self.gps_vel_ned[1])
            ve = int(self.gps_vel_ned[2])
            
            #print(latitude, longitude)
            
            lat_h = int(latitude * 1e6)
            lat_l = int((latitude * 1e6 - lat_h) * 1e8)
        
            lon_h = int(longitude * 1e6)
            lon_l = int((longitude * 1e6 - lon_h) * 1e8)
            
            
            gps_lat_msg_struct = can_dbc.get_message_by_name('GPS_Lati')
            gps_lat_msg_body = gps_lat_msg_struct.encode( \
                {'Lati_h': lat_h,\
                 'Lati_l': lat_l})
            
            gps_lat_msg = can.Message(arbitration_id = gps_lat_msg_struct.frame_id, 
                                              data=gps_lat_msg_body)
            can_bus.send(gps_lat_msg)
            
            self.ctrl_command[2] = lat_h
            self.ctrl_command[3] = lat_l
    
    
            gps_lon_msg_struct = can_dbc.get_message_by_name('GPS_Long')
            gps_lon_msg_body = gps_lon_msg_struct.encode( \
                {'Long_h': lon_h,\
                 'Long_l': lon_l})
            
            gps_lon_msg = can.Message(arbitration_id = gps_lon_msg_struct.frame_id, 
                                              data=gps_lon_msg_body)
            can_bus.send(gps_lon_msg)

            self.ctrl_command[4] = lon_h
            self.ctrl_command[5] = lon_l





            gps_vn_msg_struct = can_dbc.get_message_by_name('GPS_VelN')
            gps_vn_msg_body = gps_vn_msg_struct.encode( \
                {'GPS_VelN': vn})
            
            gps_vn_msg = can.Message(arbitration_id = gps_vn_msg_struct.frame_id, 
                                              data=gps_vn_msg_body)
            can_bus.send(gps_vn_msg)

            gps_ve_msg_struct = can_dbc.get_message_by_name('GPS_VelE')
            gps_ve_msg_body = gps_ve_msg_struct.encode( \
                {'GPS_VelE': ve})
            
            gps_ve_msg = can.Message(arbitration_id = gps_ve_msg_struct.frame_id, 
                                              data=gps_ve_msg_body)
            can_bus.send(gps_ve_msg)


            self.ctrl_command[6] = vn
            self.ctrl_command[7] = ve
            
            
            command_csv = ','.join(['%f' % value for value in self.ctrl_command])
            
            ts = time.time()
            can_send_log_file.write('%f,' % ts)
            can_send_log_file.write(command_csv)
            can_send_log_file.write('\n')
            can_send_log_file.flush()
            
            
            
            time.sleep(0.01)
    
    
    def v2v_send_thread(self):
        
        print('v2v_send_thread started')
    
        
        #dst_ips = ["192.168.0.100", "192.168.0.11", "192.168.0.12", "192.168.0.13"]
        dst_ips = ["192.168.137.1", "192.168.137.2", "192.168.137.3"]
        dst_port = 15000
        
        #src_ip = "192.168.0.1%d" % self.vid
        src_ip = "192.168.137.%d" % self.vid
        src_port = 15000 + self.vid
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
        sock.bind((src_ip, src_port))
    
        #msg_array = np.random.rand(17)
    
        
        msg_array = np.zeros(V2V_N, dtype=np.float64)
    
    
        while not self.stop:
            
            for dst_ip in dst_ips:

                msg_array[0] = self.vid
                msg_array[1] = time.time()
                msg_array[2] = self.vehicle_speed_dsr
                msg_array[3] = self.vehicle_steer_dsr
                
                msg_array[4:20] = self.imu_sample
                msg_array[20] = self.gps_pos_llh[1]
                msg_array[21] = self.gps_pos_llh[2]
                msg_array[22] = self.gps_vel_ned[1]
                msg_array[23] = self.gps_vel_ned[2]
                
                
                # TODO: IMU and GPS
    
                sock.sendto(msg_array.tobytes(), (dst_ip, dst_port))
        
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
            
            if sender_id >= 0 and sender_id < NR_VEHICLES:
                
                self.v2v_records[sender_id] = v2v_record
            
            
            v2v_csv = ','.join(['%f' % value for value in v2v_record])
            
            v2v_log.write(v2v_csv)
            v2v_log.write('\n')
    

    def v2i_send_thread(self):
        
        print('v2i_send_thread started')
    
        
        dst_ip = "192.168.1.5"
        dst_port = 16000
        
        
        src_ip = "192.168.1.%d" % (self.vid + 10)
        src_port = 16000 + self.vid
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
        sock.bind((src_ip, src_port))
    
        #msg_array = np.random.rand(17)
    
        
        
        v2i_vstate_msg = self.v2i_vstate_msg
    
        while not self.stop:
            
            time.sleep(0.1);
            
            v2i_vstate_msg[0] = self.vid
            v2i_vstate_msg[1] = time.time()
            v2i_vstate_msg[2] = self.gps_pos_llh[0]
            
            # chassis data
            
            chassis_columns = [0, 1, 2, 3, 4, 5, 8, 12, 16, 20, 23, 25]
            
            for i in range(12):
            
                col = chassis_columns[i]
                
                v2i_vstate_msg[i + 3] = self.vehicle_state[col]
            
            v2i_vstate_msg[15:31] = self.imu_sample
            v2i_vstate_msg[31:36] = self.gps_baseline_ned[1:6]
            v2i_vstate_msg[36:41] = self.gps_vel_ned[1:6]
            v2i_vstate_msg[41:46] = self.gps_pos_llh[1:6]
            
            
            try:
            
                sock.sendto(v2i_vstate_msg.tobytes(), (dst_ip, dst_port))


            except:
                
                continue
        
            
        
        pass
    
    
    def v2i_recv_thread(self):
        
        print('v2i_recv_thread started')
        

        # TODO:
        
        while not self.stop:
            
            
            time.sleep(1)

    
    def keyboard_thread(self):
        
        print('keyboard thread started.')
        
        while not self.stop:
            
            key = input('->')
            
            if key == 'exit':
                
                self.stop = True
                
            time.sleep(0.1)
        

    def process_js_event(self, ts, jsev_value, jsev_type, jsev_number):
        
        if jsev_type == 0x01:
            
            #print('Button event: ts=%u, number=%u, value=%d' % (ts, jsev_number, jsev_value))
            
            if jsev_number == 9:
                
                self.stop = True
            
            pass
        
        elif jsev_type == 0x02:
            
            #print('Axis event: ts=%u, number=%u, value=%d' % (ts, jsev_number, jsev_value))
            
            if jsev_number == 0:
                
                self.js_steer_dsr = jsev_value * 150 // 32768
            
            if jsev_number == 1:
                
                self.js_speed_dsr = -jsev_value * 150 // 32768
            
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
            
            time.sleep(0.1)
    
    
    def drive_by_wire(self):
        

        while not self.stop:
            
            # CAUTION: vehicle with ID == 1 is always the leader.
            
            if self.mode == 'v2v':
            
                self.vehicle_speed_dsr = self.v2v_records[1][2]
                self.vehicle_steer_dsr = self.v2v_records[1][3]
                
            if self.mode == 'js':
                
                self.vehicle_speed_dsr = self.js_speed_dsr
                self.vehicle_steer_dsr = self.js_steer_dsr
                
            time.sleep(0.01);


    def main_loop(self):
        
        threads = []

        x_keyboard = threading.Thread(target=self.keyboard_thread)
        threads.append(x_keyboard)
        x_keyboard.start()
          
        
        if self.mode == 'js':
              
            x_js = threading.Thread(target=self.js_thread)
            threads.append(x_js)
            x_js.start()
          
  
  
          
              
#         x_gps = threading.Thread(target=self.gps_thread)
#         threads.append(x_gps)
#         x_gps.start()
#            
#    
#         x_as = threading.Thread(target=self.arduino_sensor_thread)
#         threads.append(x_as)
#         x_as.start()
    
        x_vcan_recv = threading.Thread(target=self.chassis_can_recv_thread)
        threads.append(x_vcan_recv)
        x_vcan_recv.start()
      
        x_vcan_send = threading.Thread(target=self.chassis_can_send_thread)
        threads.append(x_vcan_send)
        x_vcan_send.start()

 
#         x_v2v_send = threading.Thread(target=self.v2v_send_thread)
#         threads.append(x_v2v_send)
#         x_v2v_send.start()
#     
#         x_v2v_recv = threading.Thread(target=self.v2v_recv_thread)
#         threads.append(x_v2v_recv)
#         x_v2v_recv.start()
        
#         x_v2i_send = threading.Thread(target=self.v2i_send_thread)
#         threads.append(x_v2i_send)
#         x_v2i_send.start()
#     
#         x_v2i_recv = threading.Thread(target=self.v2i_recv_thread)
#         threads.append(x_v2i_recv)
#         x_v2i_recv.start()

       
#         x_drive = threading.Thread(target=self.drive_by_wire)
#         threads.append(x_drive)
#         x_drive.start()
        
        
        while not self.stop:
            
            time.sleep(1)
            
        time.sleep(1)


if __name__ == '__main__':
    
    agv = AGVPi()
    
    agv.main_loop()
    
    pass






