'''

AGV management and v2v code running on the Raspberry Pi.


Created on Oct 1, 2019

@author: duolu

Flocking TODO:

    * Pi to New Eagle
    
        ** GPS time
        ** GPS vn, ve
        ** GPS pn, pe




'''

import os
import sys
import time
import socket
import struct
import threading
from multiprocessing import Process, Queue

import json

import math
from matplotlib.ft2font import LOAD_VERTICAL_LAYOUT
import numpy as np

# packages needed by the CAN bus, 
import can
import cantools

# packages needed by IMU over Arduino
import serial

# WGS-84 to local coordinate translation
from utils import *

# Initialize CAN interface
os.system('sudo /sbin/ip link set can0 up type can bitrate 500000')


# global variables

CONFIG_FILE_NAME = './conf/agvpi_flocking.conf'

# CAUTION: vehicle ID 0 is reserved, and hence, we have vehicle 0, 1, 2, 3
NR_VEHICLES = 4

# 1 for vehicle id, 1 for ts, 2 for control, 16 for IMU, 7 for GPS, in total 27
V2V_N = 27

# 1 vid, 2 timestamps, 12 chassis states, 16 IMU states, 15 GPS states
V2I_N = 1 + 2 + 12 + 16 + 15


class AGVCANTxAgent(object):
    
    
    def __init__(self, q_can_send, q_can_send_ctrl):
        
        self.q_can_send = q_can_send
        self.q_can_send_ctrl = q_can_send_ctrl
        
        self.stop = False
        
        self.ctrl_command = np.zeros(40, dtype=np.float64)
        
        self.init_with_config_file()
        


    def init_with_config_file(self):
        
        with open(CONFIG_FILE_NAME, mode='r') as fd:
        
            config_str = fd.read()
        
            config = json.loads(config_str)
            
            self.dbc_file = config['dbc_file']
            
            #print(self.vid, self.mode)
    
    
    def deliver_can_command_msg(self, command):
        
        gps_time = command[4]
        latitude = command[5]
        longitude = command[6]
        vn = int(command[7])
        ve = int(command[8])
        pn = int(command[9])
        pe = int(command[10])

        #latitude = 37.77405700243414
        #longitude = -122.41683960033353

        print('Lat and lon read successfully!')
        
        
        lat_h = int(latitude * 1e6)
        lat_l = int((latitude * 1e6 - lat_h) * 1e8)
    
        lon_h = int(longitude * 1e6)
        lon_l = int((longitude * 1e6 - lon_h) * 1e8)

        print(latitude, lat_h, lat_l)
        print(longitude, lon_h, lon_l)


        # GPS time

        gps_time_msg_struct = self.can_dbc.get_message_by_name('GPS_Time')
        gps_time_msg_body = gps_time_msg_struct.encode( \
            {'GPS_Time': gps_time})

        gps_time_msg = can.Message(arbitration_id = gps_time_msg_struct.frame_id, 
                                          data=gps_time_msg_body)
        self.can_bus.send(gps_time_msg)
        
        
        # GPS latitude / longitude
        
        gps_lat_msg_struct = self.can_dbc.get_message_by_name('GPS_Lati')
        gps_lat_msg_body = gps_lat_msg_struct.encode( \
            {'Lati_h': lat_h,\
             'Lati_l': lat_l})
        
        gps_lat_msg = can.Message(arbitration_id = gps_lat_msg_struct.frame_id, 
                                          data=gps_lat_msg_body)
        self.can_bus.send(gps_lat_msg)
        

        gps_lon_msg_struct = self.can_dbc.get_message_by_name('GPS_Long')
        gps_lon_msg_body = gps_lon_msg_struct.encode( \
            {'Long_h': lon_h,\
             'Long_l': lon_l})
        
        gps_lon_msg = can.Message(arbitration_id = gps_lon_msg_struct.frame_id, 
                                          data=gps_lon_msg_body)
        self.can_bus.send(gps_lon_msg)


        # GPS vn / ve
        
        gps_vel_msg_struct = self.can_dbc.get_message_by_name('GPS_Vel')
        gps_vel_msg_body = gps_vel_msg_struct.encode( \
            {'GPS_VelN': vn,\
             'GPS_VelE': ve})
        
        gps_vel_msg = can.Message(arbitration_id = gps_vel_msg_struct.frame_id, 
                                          data=gps_vel_msg_body)
        self.can_bus.send(gps_vel_msg)


        # GPS pn / pe

        gps_pos_msg_struct = self.can_dbc.get_message_by_name('GPS_Pos')
        gps_pos_msg_body = gps_pos_msg_struct.encode( \
            {'GPS_PosN': pn,\
             'GPS_PosE': pe})
        
        gps_pos_msg = can.Message(arbitration_id = gps_pos_msg_struct.frame_id, 
                                          data=gps_pos_msg_body)
        self.can_bus.send(gps_pos_msg)



        # # GPS latitude / longitude
        
        # gps_lat_msg_struct = self.can_dbc.get_message_by_name('GPS_Lati')
        # gps_lat_msg_body = gps_lat_msg_struct.encode( \
        #     {'Lati_h': lat_h,\
        #      'Lati_l': lat_l})
        
        # gps_lat_msg = can.Message(arbitration_id = gps_lat_msg_struct.frame_id, 
        #                                   data=gps_lat_msg_body)
        # self.can_bus.send(gps_lat_msg)
        # print('Lat sent!')
        

        # gps_lon_msg_struct = self.can_dbc.get_message_by_name('GPS_Long')
        # gps_lon_msg_body = gps_lon_msg_struct.encode( \
        #     {'Long_h': lon_h,\
        #      'Long_l': lon_l})
        
        # gps_lon_msg = can.Message(arbitration_id = gps_lon_msg_struct.frame_id, 
        #                                   data=gps_lon_msg_body)
        # self.can_bus.send(gps_lon_msg)
        # print('Lon sent!')
        

        # add by Jialin on 05/11/2022
        # self.stop = True



    # def deliver_can_command_msg(self, command):
        
    #     # drive by wire
    
    #     test_drive_msg_struct = self.can_dbc.get_message_by_name('Test_Drive')
    #     test_drive_msg_body = test_drive_msg_struct.encode( \
    #         {'v_speed_dsr': command[0], \
    #          'steer_dsr': command[1]})
        
    #     test_drive_msg = can.Message(arbitration_id = test_drive_msg_struct.frame_id, 
    #                                       data=test_drive_msg_body)
       
    #     self.can_bus.send(test_drive_msg)

    #     # control mode

    #     ctrl_mode_msg_struct = self.can_dbc.get_message_by_name('Control_Mode')
    #     ctrl_mode_msg_body = ctrl_mode_msg_struct.encode( \
    #         {'Control_Enable': command[2], \
    #          'E_Brake': command[3]})
        
    #     ctrl_mode_msg = can.Message(arbitration_id = ctrl_mode_msg_struct.frame_id, 
    #                                       data=ctrl_mode_msg_body)
       
    #     self.can_bus.send(ctrl_mode_msg)
        
        
    #     # local vehicle (GPS / IMU)
        
    #     gps_time = command[4]
        
    #     latitude = command[5]
    #     longitude = command[6]

    #     vn = int(command[7])
    #     ve = int(command[8])

    #     pn = int(command[9])
    #     pe = int(command[10])

    #     imu_yaw_rate = int(command[11] * 1000)

        
    #     lat_h = int(latitude * 1e6)
    #     lat_l = int((latitude * 1e6 - lat_h) * 1e8)
    
    #     lon_h = int(longitude * 1e6)
    #     lon_l = int((longitude * 1e6 - lon_h) * 1e8)


    #     # GPS time

    #     gps_time_msg_struct = self.can_dbc.get_message_by_name('GPS_Time')
    #     gps_time_msg_body = gps_time_msg_struct.encode( \
    #         {'GPS_Time': gps_time})

    #     gps_time_msg = can.Message(arbitration_id = gps_time_msg_struct.frame_id, 
    #                                       data=gps_time_msg_body)
    #     self.can_bus.send(gps_time_msg)
        
        
    #     # GPS latitude / longitude
        
    #     gps_lat_msg_struct = self.can_dbc.get_message_by_name('GPS_Lati')
    #     gps_lat_msg_body = gps_lat_msg_struct.encode( \
    #         {'Lati_h': lat_h,\
    #          'Lati_l': lat_l})
        
    #     gps_lat_msg = can.Message(arbitration_id = gps_lat_msg_struct.frame_id, 
    #                                       data=gps_lat_msg_body)
    #     self.can_bus.send(gps_lat_msg)
        

    #     gps_lon_msg_struct = self.can_dbc.get_message_by_name('GPS_Long')
    #     gps_lon_msg_body = gps_lon_msg_struct.encode( \
    #         {'Long_h': lon_h,\
    #          'Long_l': lon_l})
        
    #     gps_lon_msg = can.Message(arbitration_id = gps_lon_msg_struct.frame_id, 
    #                                       data=gps_lon_msg_body)
    #     self.can_bus.send(gps_lon_msg)


    #     # GPS vn / ve
        
    #     gps_vel_msg_struct = self.can_dbc.get_message_by_name('GPS_Vel')
    #     gps_vel_msg_body = gps_vel_msg_struct.encode( \
    #         {'GPS_VelN': vn,\
    #          'GPS_VelE': ve})
        
    #     gps_vel_msg = can.Message(arbitration_id = gps_vel_msg_struct.frame_id, 
    #                                       data=gps_vel_msg_body)
    #     self.can_bus.send(gps_vel_msg)


    #     # GPS pn / pe

    #     gps_pos_msg_struct = self.can_dbc.get_message_by_name('GPS_Pos')
    #     gps_pos_msg_body = gps_pos_msg_struct.encode( \
    #         {'GPS_PosN': pn,\
    #          'GPS_PosE': pe})
        
    #     gps_pos_msg = can.Message(arbitration_id = gps_pos_msg_struct.frame_id, 
    #                                       data=gps_pos_msg_body)
    #     self.can_bus.send(gps_pos_msg)


    #     # IMU yaw rate

    #     r_in_msg_struct = self.can_dbc.get_message_by_name('IMUsingals')
    #     r_in_msg_body = r_in_msg_struct.encode( \
    #         {'r_in': imu_yaw_rate})
        
    #     r_in_msg = can.Message(arbitration_id = r_in_msg_struct.frame_id, 
    #                                       data=r_in_msg_body)
    #     self.can_bus.send(r_in_msg)

        
        
    #     # flocking, first vehicle
        
    #     gps_time_ov1 = command[20]
        
    #     gps_vn_ov1 = command[25]
    #     gps_ve_ov1 = command[26]

    #     gps_pn_ov1 = command[23]
    #     gps_pe_ov1 = command[24]
        
    #     gps_time_ov1_msg_struct = self.can_dbc.get_message_by_name('GPS_Time_OV1')
    #     gps_time_ov1_msg_body = gps_time_ov1_msg_struct.encode( \
    #         {'GPS_Time_OV1': gps_time_ov1})
        
    #     gps_time_ov1_msg = can.Message(arbitration_id = gps_time_ov1_msg_struct.frame_id, 
    #                                       data=gps_time_ov1_msg_body)
    #     self.can_bus.send(gps_time_ov1_msg)
        
        
    #     # GPS vn / ve
        
    #     gps_vel_ov1_msg_struct = self.can_dbc.get_message_by_name('GPS_Vel_OV1')
    #     gps_vel_ov1_msg_body = gps_vel_ov1_msg_struct.encode( \
    #         {'GPS_VelN_OV1': gps_vn_ov1,\
    #          'GPS_VelE_OV1': gps_ve_ov1})
        
    #     gps_vel_ov1_msg = can.Message(arbitration_id = gps_vel_ov1_msg_struct.frame_id, 
    #                                       data=gps_vel_ov1_msg_body)
    #     self.can_bus.send(gps_vel_ov1_msg)


    #     # GPS pn / pe

    #     gps_pos_ov1_msg_struct = self.can_dbc.get_message_by_name('GPS_Pos_OV1')
    #     gps_pos_ov1_msg_body = gps_pos_ov1_msg_struct.encode( \
    #         {'GPS_PosN_OV1': gps_pn_ov1,\
    #          'GPS_PosE_OV1': gps_pe_ov1})
        
    #     gps_pos_ov1_msg = can.Message(arbitration_id = gps_pos_ov1_msg_struct.frame_id, 
    #                                       data=gps_pos_ov1_msg_body)
    #     self.can_bus.send(gps_pos_ov1_msg)
        
        
        
    #     # flocking, first vehicle
        
    #     gps_time_ov2 = command[30]
        
    #     gps_vn_ov2 = command[35]
    #     gps_ve_ov2 = command[36]

    #     gps_pn_ov2 = command[33]
    #     gps_pe_ov2 = command[34]
        
    #     gps_time_ov2_msg_struct = self.can_dbc.get_message_by_name('GPS_Time_OV2')
    #     gps_time_ov2_msg_body = gps_time_ov2_msg_struct.encode( \
    #         {'GPS_Time_OV2': gps_time_ov2})
        
    #     gps_time_ov2_msg = can.Message(arbitration_id = gps_time_ov2_msg_struct.frame_id, 
    #                                       data=gps_time_ov2_msg_body)
    #     self.can_bus.send(gps_time_ov2_msg)
        
        
    #     # GPS vn / ve
        
    #     gps_vel_ov2_msg_struct = self.can_dbc.get_message_by_name('GPS_Vel_OV2')
    #     gps_vel_ov2_msg_body = gps_vel_ov2_msg_struct.encode( \
    #         {'GPS_VelN_OV2': gps_vn_ov2,\
    #          'GPS_VelE_OV2': gps_ve_ov2})
        
    #     gps_vel_ov2_msg = can.Message(arbitration_id = gps_vel_ov2_msg_struct.frame_id, 
    #                                       data=gps_vel_ov2_msg_body)
    #     self.can_bus.send(gps_vel_ov2_msg)


    #     # GPS pn / pe

    #     gps_pos_ov2_msg_struct = self.can_dbc.get_message_by_name('GPS_Pos_OV2')
    #     gps_pos_ov2_msg_body = gps_pos_ov2_msg_struct.encode( \
    #         {'GPS_PosN_OV2': gps_pn_ov2,\
    #          'GPS_PosE_OV2': gps_pe_ov2})
        
    #     gps_pos_ov2_msg = can.Message(arbitration_id = gps_pos_ov2_msg_struct.frame_id, 
    #                                       data=gps_pos_ov2_msg_body)
    #     self.can_bus.send(gps_pos_ov2_msg)

    
    def run(self):
        
        print('AGV CAN Tx Agent started.')
        
        command_log_file_raw = open('./conf/command_can.csv', 'w')


        self.can_bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
        self.can_dbc = cantools.database.load_file(self.dbc_file)
        
        
        while not self.stop:
            print('Enter while loop!')

            self.ctrl_command = self.q_can_send.get()
            print('ctrl_comand=', self.ctrl_command)

            self.deliver_can_command_msg(self.ctrl_command)
        
        
            ts = time.time()
            
            msg_csv = ','.join([str(value) for value in self.ctrl_command])
        
            command_log_file_raw.write('%f,' % ts)
            command_log_file_raw.write(msg_csv)
            command_log_file_raw.write('\n')
            command_log_file_raw.flush()

    
            if not self.q_can_send_ctrl.empty():
                
                print('q_can_send_ctrl triggered')
                self.stop = True
    



        print('AGV CAN Tx Agent stopped.')


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


    def __init__(self, 
                 q_can_recv, q_can_recv_ctrl, 
                 q_can_send, q_can_send_ctrl):
        
        # global initial configuration
        
        self.vid = 0
        self.mode = 'none'
        
        self.exp_start = 0
        self.ctrl_enable = 0
        self.emergency_brake = 0
        
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
        self.pos_llh = (0,0,0,0,0,0,0)
        self.gps_baseline_pos_ned = (0,0,0,0,0,0,0)
        self.gps_baseline_vel_ned = (0,0,0,0,0,0,0)
        
        self.gps_fixed_origin_pos_ned = np.zeros(3, np.float64)
        self.lat0 = 33.30904900126126
        self.lon0 = -111.67331256803944
        self.height0 = 377.998120475112


        # Chassis and vehicle controller states
        self.vehicle_speed_dsr = 0
        self.vehicle_steer_dsr = 0
        self.vehicle_state = np.zeros(34, dtype=np.float64)
        
        self.ctrl_state = np.zeros(24 + 20, dtype=np.float64)
        
        self.ctrl_command = np.zeros(40, dtype=np.float64)


        self.q_can_recv = q_can_recv
        self.q_can_recv_ctrl = q_can_recv_ctrl

        self.q_can_send = q_can_send
        self.q_can_send_ctrl = q_can_send_ctrl


        # V2V states

        # these are the most recent V2V records received from other vehicles
        # CAUTION: the actual transmitted message is in binary format (C struct),
        # not numpy array.
        self.v2v_records = np.zeros((NR_VEHICLES, V2V_N), np.float64)
        

        # V2I states

        # most recently V2I command (received) and state report (sent)
        # CAUTION: the actual transmitted message is in binary format (C struct),
        # not numpy array.
        self.v2i_command_msg = np.zeros(4, dtype=np.float64)
        self.v2i_vstate_msg = np.zeros(V2I_N, dtype=np.float64)

        
        pass


    def init_with_config_file(self):
        
        with open(CONFIG_FILE_NAME, mode='r') as fd:
        
            config_str = fd.read()
        
            config = json.loads(config_str)
            
            self.vid = int(config['vid'])
            self.mode = config['mode']
            self.dbc_file = config['dbc_file']
            
            #print(self.vid, self.mode)
        
        pass


    def log_csv_record(self, fd, ts, tup):
        
        msg_csv = ','.join([str(value) for value in tup])
        
        fd.write('%f,' % ts)
        fd.write(msg_csv)
        fd.write('\n')
        fd.flush()


    def gps_thread(self):
        
        print("GPS thread started.")
        
        log_file_raw_record = open('./log/gps_raw_record', 'w')
        
        log_pos_llh = open('./log/gps_pos_llh.csv', 'w')
        log_baseline_ned = open('./log/gps_baseline_pos_ned.csv', 'w')
        log_vel_ned = open('./log/gps_baseline_vel_ned.csv', 'w')
    
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
                    self.pos_llh = pos_llh
                    
                    self.gps_fixed_origin_pos_ned = wgs84_llh_to_ned(
                        pos_llh[1], pos_llh[2], pos_llh[3], 
                        self.lat0, self.lon0, self.height0)
                    
                    #print(pos_llh[1], pos_llh[2], pos_llh[3], 
                    #      self.lat0, self.lon0, self.height0)
                    
                    #print(self.gps_fixed_origin_pos_ned)
                    
                    self.gps_fixed_origin_pos_ned \
                        = self.gps_fixed_origin_pos_ned * 1000
                    
                    #print(self.gps_fixed_origin_pos_ned)
                    
                    self.log_csv_record(log_pos_llh, ts, pos_llh)
    
                elif msg_type == 0x0211: # MSG_POS_LLH_COV
                    
                    pos_llh_cov = struct.unpack('<IdddffffffBBH', payload)

                elif msg_type == 0x020B: # MSG_BASELINE_ECEF
                    
                    baseline_ecef = struct.unpack('<IiiiHBBH', payload)
    
                elif msg_type == 0x020C: # MSG_BASELINE_NED
                    
                    baseline_ned = struct.unpack('<IiiiHHBBH', payload)
                    self.gps_baseline_pos_ned = baseline_ned
                    
                    self.log_csv_record(log_baseline_ned, ts, baseline_ned)
                
                elif msg_type == 0x020D: # MSG_VEL_ECEF
                    
                    vel_ned_ecef = struct.unpack('<IiiiHBBH', payload)
                    
                elif msg_type == 0x0215: # MSG_VEL_ECEF_COV
                    
                    vel_ned_ecef_cov = struct.unpack('<IiiiffffffBBH', payload)
    
                elif msg_type == 0x020E: # MSG_VEL_NED
                    
                    vel_ned = struct.unpack('<IiiiHHBBH', payload)
                    self.gps_baseline_vel_ned = vel_ned
                    
                    self.log_csv_record(log_vel_ned, ts, vel_ned)
                
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



    def construct_ctrl_command(self, command):
        
        command[0] = self.vehicle_speed_dsr
        command[1] = self.vehicle_steer_dsr
        
        # control enable, sent by the control center
        command[2] = self.ctrl_enable
        
        # Emergency Brake, currently always 0
        command[3] = 0
        
        command[4] = self.pos_llh[0]
        
        command[5] = self.pos_llh[1]
        command[6] = self.pos_llh[2]
        
        command[7] = self.gps_baseline_vel_ned[1]
        command[8] = self.gps_baseline_vel_ned[2]
        
        command[9] = self.gps_baseline_pos_ned[1]
        command[10] = self.gps_baseline_pos_ned[2]

        # CAUTION: position must be in milimeter
        #command[9] = self.gps_fixed_origin_pos_ned[0]
        #command[10] = self.gps_fixed_origin_pos_ned[1]

        command[11] = self.imu_sample[5]
        
        # flocking
        j = 0
        for vid_other in range(1, NR_VEHICLES):
            
            if vid_other == self.vid:
                
                continue
            
            idx_start = 20 + j * 10
            idx_end = idx_start + 7
        
            # GPS states of the first other vehicle
            command[idx_start:idx_end] = self.v2v_records[vid_other, 20:27]
        
            j += 1
        
        
        pass


    def chassis_can_send_thread(self):
        
        print('chassis_can_send_thread started.')

        ts_last = time.time()
        
        while not self.stop:
            
            ts = time.time()
            
            if ts - ts_last > 0.05:
                
                ts_last = ts
            
                self.construct_ctrl_command(self.ctrl_command)
            
                self.q_can_send.put(self.ctrl_command)
            
            
            time.sleep(0.05)
            
            pass
        
        
        self.q_can_send_ctrl.put(np.zeros(1))
        self.construct_ctrl_command(self.ctrl_command)
        self.q_can_send.put(self.ctrl_command)
        
        print('chassis_can_send_thread stopped.')
  

    # def main_loop(self):
        
    #     while not self.stop:
                
    #         time.sleep(1)
                
    #     time.sleep(1)
            
    #     print('main exit')

    def main_loop(self):
        
        threads = []

        x_gps = threading.Thread(target=self.gps_thread)
        threads.append(x_gps)
        x_gps.start()

        x_vcan_send = threading.Thread(target=self.chassis_can_send_thread)
        threads.append(x_vcan_send)
        x_vcan_send.start()

        while not self.stop:
            time.sleep(1)
            
        time.sleep(1)
        
        print('main exit')



def can_send_process(q_send, q_send_ctrl):

    tx_agent = AGVCANTxAgent(q_send, q_send_ctrl)
    
    tx_agent.run()

    pass


if __name__ == '__main__':
    
    
    q_can_recv = Queue()
    q_can_recv_ctrl = Queue()
    
    q_can_send = Queue()
    q_can_send_ctrl = Queue()

    # lat = 37.77405700234567
    # lon = -122.41683960123456
    # for i in range(1):
    #     q_can_send.put([lat,lon])
    
    # p1 = Process(target=can_recv_process, args=(q_can_recv, q_can_recv_ctrl))
    
    p2 = Process(target=can_send_process, args=(q_can_send, q_can_send_ctrl))
    
    # p1.start()
    p2.start()
    
    agv = AGVPi(q_can_recv, q_can_recv_ctrl, q_can_send, q_can_send_ctrl)
    print('Enter main loop!')
    
    agv.main_loop()
    print('Exit main loop!')
    
#     p1.terminate()
#     p2.terminate()
    
    pass

