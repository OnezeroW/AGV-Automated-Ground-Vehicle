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

import shutil
import os
import sys
import time
import socket
import struct
import threading
from multiprocessing import Process, Queue

import json

import math
import numpy as np

# packages needed by the CAN bus, 
import can
import cantools

# packages needed by IMU over Arduino
import serial

# WGS-84 to local coordinate translation
from utils import *


# global variables

CONFIG_FILE_NAME = './agv_pi.conf'
FLOCKING_CONFIG_FILE_NAME = './agv_pi_flocking.conf'

# CAUTION: vehicle ID 0 is reserved, and hence, we have vehicle 0, 1, 2, 3
NR_VEHICLES = 4

# 1 for vehicle id, 1 for ts, 2 for control, 16 for IMU, 7 for GPS, in total 27
V2V_N = 12

# 1 vid, 2 timestamps, 12 chassis states, 16 IMU states, 15 GPS states
V2I_N = 1 + 2 + 12 + 16 + 15



# from the CAN recv process to the main process, in q_can_recv
VEHICLE_STATE_N = 34
CTRL_STAT_N = 24 + 20

COMMAND_N = 40


class FlockingPara(object):
    
    def __init__(self):
        
        self.sigma = 0.1
        
        self.r_a = 10.0
        self.h_alpha = 0.5
        self.d_a = 2.0
        self.a_alpha = 5
        self.b_alpha = 5

        self.c1_alpha = 1.0
        self.c2_alpha = 1.0

        self.r_b = 5.0
        self.h_beta = 0.5

        self.c1_beta = 1.0
        self.c2_beta = 1.0

        
        self.c1_gamma = 1.0
        self.c2_gamma = 1.0

        self.c_alpha = 0.0 # 1.0
        self.c_beta = 1.0  # 1.0
        self.c_gamma = 0.25


    def init_with_config_file(self):


        with open(FLOCKING_CONFIG_FILE_NAME, mode='r') as fd:
        
            config_str = fd.read()
        
            config = json.loads(config_str)
            
            self.sigma = config['sigma']
            
            self.r_a = config['r_a']
            self.h_alpha = config['h_alpha']
            self.d_a = config['d_a']
            self.a_alpha = config['a_alpha']
            self.b_alpha = config['b_alpha']
    
            self.c1_alpha = config['c1_alpha']
            self.c2_alpha = config['c2_alpha']
    
            self.r_b = config['r_b']
            self.h_beta = config['h_beta']
    
            self.c1_beta = config['c1_beta']
            self.c2_beta = config['c2_beta']
    
            
            self.c1_gamma = config['c1_gamma']
            self.c2_gamma = config['c2_gamma']
    
            self.c_alpha = config['c_alpha']
            self.c_beta = config['c_beta']
            self.c_gamma = config['c_gamma']

            self.ob_x = config['ob_x']
            self.ob_y = config['ob_y']
            self.ob_r = config['ob_r']

            self.wall_1_y = config['wall_1_y']
            
            self.wall_2_y = config['wall_2_y']

            self.leader_x = config['leader_x']
            self.leader_y = config['leader_y']
            self.leader_vx = config['leader_vx']
            self.leader_vy = config['leader_vy']
            
            self.leader_mode = config['leader_mode']
            self.leader_vid = int(config['leader_vid'])


    pass


class AGVCANRxAgent(object):
    '''The CAN receive process.
    '''
    
    def __init__(self, q_can_recv, q_can_recv_ctrl):
        
        self.q_can_recv = q_can_recv
        self.q_can_recv_ctrl = q_can_recv_ctrl
        
        self.stop = False
        
        
        self.vehicle_state = np.zeros(VEHICLE_STATE_N, dtype=np.float64)
        self.ctrl_state = np.zeros(CTRL_STAT_N, dtype=np.float64)
        
        self.init_with_config_file()
        
        pass

    def init_with_config_file(self):
        
        with open(CONFIG_FILE_NAME, mode='r') as fd:
        
            config_str = fd.read()
        
            config = json.loads(config_str)
            
            self.dbc_file = config['dbc_file']
            
            #print(self.vid, self.mode)
        
        pass

    
    def process_can_report_msg(self, msg):
        
        
        mid = int(msg.arbitration_id)

        if mid == 0x0C000101: # steering control
            
            steering_ctrl = struct.unpack('>BhhhB', msg.data)
            
            self.vehicle_state[0] = steering_ctrl[1]    # steering 1 ctrl (angle)
            self.vehicle_state[1] = steering_ctrl[2]    # steering 2 ctrl (angle)
            self.vehicle_state[2] = steering_ctrl[3]    # motor 1 ctrl (torque)
        
        elif mid == 0x0C000102: # driving control
            
            driving_ctrl = struct.unpack('>hhhBB', msg.data)
            
            self.vehicle_state[3] = driving_ctrl[0]    # motor 2 ctrl (torque)
            self.vehicle_state[4] = driving_ctrl[1]    # motor 3 ctrl (torque)
            self.vehicle_state[5] = driving_ctrl[2]    # motor 4 ctrl (torque)
        
        elif mid == 0x0B520101: # wheel 1 torque feedback
            
            d11 = struct.unpack('>BBhhH', msg.data)
            
            self.vehicle_state[6] = d11[1]  # pre status
            self.vehicle_state[7] = d11[2]  # torque
            self.vehicle_state[8] = d11[3]  # error

        elif mid == 0x0B520201: # wheel 1 speed feedback
            
            d12 = struct.unpack('>BhhBH', msg.data)
            
            self.vehicle_state[9] = d12[1]  # speed
            self.vehicle_state[10] = d12[2] # angle
            self.vehicle_state[11] = d12[3] # direction

        elif mid == 0x0B520102: # wheel 2 torque feedback
            
            d21 = struct.unpack('>BBhhH', msg.data)
            
            self.vehicle_state[12] = d21[1]
            self.vehicle_state[13] = d21[2]
            self.vehicle_state[14] = d21[3]

        elif mid == 0x0B520202: # wheel 2 speed feedback
            
            d22 = struct.unpack('>BhhBH', msg.data)
            
            self.vehicle_state[15] = d22[1]
            self.vehicle_state[16] = d22[2]
            self.vehicle_state[17] = d22[3]

        elif mid == 0x0B520103: # wheel 3 torque feedback
            
            d31 = struct.unpack('>BBhhH', msg.data)
            
            self.vehicle_state[18] = d31[1]
            self.vehicle_state[19] = d31[2]
            self.vehicle_state[20] = d31[3]

        elif mid == 0x0B520203: # wheel 3 speed feedback
            
            d32 = struct.unpack('>BhhBH', msg.data)
            
            self.vehicle_state[21] = d32[1]
            self.vehicle_state[22] = d32[2]
            self.vehicle_state[23] = d32[3]

        elif mid == 0x0B520104: # wheel 4 torque feedback
            
            d41 = struct.unpack('>BBhhH', msg.data)
            
            self.vehicle_state[24] = d41[1]
            self.vehicle_state[25] = d41[2]
            self.vehicle_state[26] = d41[3]

        elif mid == 0x0B520204: # wheel 4 speed feedback
            
            d42 = struct.unpack('>BhhBH', msg.data)
            
            self.vehicle_state[27] = d42[1]
            self.vehicle_state[28] = d42[2]
            self.vehicle_state[29] = d42[3]

        elif mid == 0x00000381: # streering 1 feedback
            
            s1 = struct.unpack('<HBBi', msg.data)
            
            self.vehicle_state[30] = s1[2]
            self.vehicle_state[31] = s1[3]

        elif mid == 0x00000382: # steering 2 feedback
            
            s2 = struct.unpack('<HBBi', msg.data)
            
            self.vehicle_state[32] = s2[2]
            self.vehicle_state[33] = s2[3]




        elif mid == 0x0A000117: # open loop control drive by wire
            
            drive_by_wire = struct.unpack('<hhhh', msg.data)
            
            self.ctrl_state[0] = drive_by_wire[0] # desired vehicle speed
            self.ctrl_state[1] = drive_by_wire[1] # desired vehicle steering angle

        elif mid == 0x0A000101: # open loop control drive by wire
            
            ctrl_mode = struct.unpack('<BBBBBBBB', msg.data)
            
            self.ctrl_state[2] = ctrl_mode[0] # controller enable
            self.ctrl_state[3] = ctrl_mode[1] # emergency brake


        elif mid == 0x0A000201: # GPS latitude
            
            gps_latitude = struct.unpack('<ii', msg.data)
            
            self.ctrl_state[4] = gps_latitude[0] # fixed point latitude, first half
            self.ctrl_state[5] = gps_latitude[1] # fixed point latitude, second half

        elif mid == 0x0A000202: # GPS longitude
            
            gps_longitude = struct.unpack('<ii', msg.data)
            
            self.ctrl_state[6] = gps_longitude[0] # fixed point longitude, first half
            self.ctrl_state[7] = gps_longitude[1] # fixed point longitude, second half

        elif mid == 0x0A000203: # GPS velocity ENU
            
            gps_vel_ned = struct.unpack('<ii', msg.data)
            
            self.ctrl_state[8] = gps_vel_ned[0] # velocity component in north
            self.ctrl_state[9] = gps_vel_ned[1] # velocity component in east

        elif mid == 0x0A000204: # GPS position ENU
            
            gps_pos_ned = struct.unpack('<ii', msg.data)
            
            self.ctrl_state[10] = gps_pos_ned[0] # position component in north
            self.ctrl_state[11] = gps_pos_ned[1] # position component in east

        elif mid == 0x00000205: # calculated speed by kinematic model
            
            vxy = struct.unpack('<hhhh', msg.data)
            
            self.ctrl_state[12] = vxy[0] # vx
            self.ctrl_state[13] = vxy[1] # vy
            self.ctrl_state[14] = vxy[2] # ax
            self.ctrl_state[15] = vxy[3] # Ay

        elif mid == 0x00000206: # calculated angles by kinematic model and speed
            
            angles = struct.unpack('<hhhh', msg.data)
            
            self.ctrl_state[16] = angles[0] # heading angle
            self.ctrl_state[17] = angles[1] # yaw angle
            self.ctrl_state[18] = angles[2] # side slip
            self.ctrl_state[19] = angles[3] # r_out

        elif mid == 0x00000207: # calculated angles by kinematic model and speed
            
            errors = struct.unpack('<hhhh', msg.data)
            
            self.ctrl_state[20] = errors[0] # ey
            self.ctrl_state[21] = errors[1] # eh

        elif mid == 0x0A000208: # calculated angles by kinematic model and speed
            
            imu_state = struct.unpack('<hhhh', msg.data)
            
            self.ctrl_state[22] = imu_state[0] # r_in
        
        elif mid == 0x0A000200: # GPS time
            
            gps_time = struct.unpack('<ii', msg.data)
            
            self.ctrl_state[23] = gps_time[0] # GPS time
        
        # flocking other vehicle 1
        
        elif mid == 0x0A000209: # GPS time
            
            flocking_u = struct.unpack('<ii', msg.data)
            
            self.ctrl_state[24] = flocking_u[0] # ux
            self.ctrl_state[25] = flocking_u[1] # uy
        
       
    
    
    
    
    def run(self):
        
        
        print('AGV CAN Rx Agent started.')
        
        chassis_log_file_raw = open('./log/chassis_can.raw', 'w')


        self.can_bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
        self.can_dbc = cantools.database.load_file(self.dbc_file)
        
        ts_last = time.time()
        
        while not self.stop:
            
            msg = self.can_bus.recv()
            
            self.process_can_report_msg(msg)
            #self.process_can_report_msg_using_dbc(msg)

            # log every raw message
            ts = time.time()
            chassis_log_file_raw.write('%f,' % ts)
            chassis_log_file_raw.write('%X, ' % msg.arbitration_id)
            chassis_log_file_raw.write(msg.data.hex())
            chassis_log_file_raw.write('\n')
            chassis_log_file_raw.flush()
        
            # report the states every 50 ms (i.e., 20 Hz)
            if ts - ts_last > 0.05:
                
                ts_last = ts
                
                states_in_q = np.concatenate((self.vehicle_state, self.ctrl_state))
                
                self.q_can_recv.put(states_in_q)
        
            if not self.q_can_recv_ctrl.empty():
                
                print('q_can_recv_ctrl triggered')
                self.stop = True
        
        
        
        
        print('AGV CAN Rx Agent stopped.')


class AGVCANTxAgent(object):
    
    
    def __init__(self, q_can_send, q_can_send_ctrl):
        
        self.q_can_send = q_can_send
        self.q_can_send_ctrl = q_can_send_ctrl
        
        self.stop = False
        
        self.ctrl_command = np.zeros(COMMAND_N, dtype=np.float64)
        
        self.init_with_config_file()
        


    def init_with_config_file(self):
        
        with open(CONFIG_FILE_NAME, mode='r') as fd:
        
            config_str = fd.read()
        
            config = json.loads(config_str)
            
            self.dbc_file = config['dbc_file']
            
            #print(self.vid, self.mode)
    
    
    def deliver_can_command_msg(self, command):
        
        # drive by wire
    
        test_drive_msg_struct = self.can_dbc.get_message_by_name('Test_Drive')
        test_drive_msg_body = test_drive_msg_struct.encode( \
            {'v_speed_dsr': command[0], \
             'steer_dsr': command[1]})
        
        test_drive_msg = can.Message(arbitration_id = test_drive_msg_struct.frame_id, 
                                          data=test_drive_msg_body)
       
        self.can_bus.send(test_drive_msg)

        # control mode

        ctrl_mode_msg_struct = self.can_dbc.get_message_by_name('Control_Mode')
        ctrl_mode_msg_body = ctrl_mode_msg_struct.encode( \
            {'Control_Enable': command[2], \
             'E_Brake': command[3]})
        
        ctrl_mode_msg = can.Message(arbitration_id = ctrl_mode_msg_struct.frame_id, 
                                          data=ctrl_mode_msg_body)
       
        self.can_bus.send(ctrl_mode_msg)
        
        
        # local vehicle (GPS / IMU)
        
        gps_time = command[4]
        
        latitude = command[5]
        longitude = command[6]

        vn = int(command[7])
        ve = int(command[8])

        pn = int(command[9])
        pe = int(command[10])

        imu_yaw_rate = int(command[11] * 1000)

        ux = int(command[12] * 1000)
        uy = int(command[13] * 1000)

        
        lat_h = int(latitude * 1e6)
        lat_l = int((latitude * 1e6 - lat_h) * 1e8)
    
        lon_h = int(longitude * 1e6)
        lon_l = int((longitude * 1e6 - lon_h) * 1e8)


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


        # IMU yaw rate

        r_in_msg_struct = self.can_dbc.get_message_by_name('IMUsingals')
        r_in_msg_body = r_in_msg_struct.encode( \
            {'r_in': imu_yaw_rate})
        
        r_in_msg = can.Message(arbitration_id = r_in_msg_struct.frame_id, 
                                          data=r_in_msg_body)
        self.can_bus.send(r_in_msg)

        
        # flocking, u_vec    
        
        u_vec_msg_struct = self.can_dbc.get_message_by_name('Flocking_U')
        u_vec_msg_body = u_vec_msg_struct.encode( \
            {'ux': ux, 'uy': uy})
        
        u_vec_msg = can.Message(arbitration_id = u_vec_msg_struct.frame_id, 
                                          data=u_vec_msg_body)
        self.can_bus.send(u_vec_msg)
        
        
        
    
    def run(self):
        
        print('AGV CAN Tx Agent started.')
        
        command_log_file_raw = open('./log/command_can.csv', 'w')


        self.can_bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
        self.can_dbc = cantools.database.load_file(self.dbc_file)
        
        
        while not self.stop:

            self.ctrl_command = self.q_can_send.get()
        
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
    and it connects to the central control panel and other vehicles.
    
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
        
        
        # load initial configuration from config file
        
        self.init_with_config_file()

        self.para = FlockingPara()
        self.para.init_with_config_file()
        
        shutil.copy('./agv_pi_flocking.conf', './log/agv_pi_flocking.conf')


        assert self.vid > 0 and self.vid <= NR_VEHICLES


        # global states

        self.stop = False
        self.ctrl_enable = 0
        self.emergency_brake = 0
        
        # joy stick controller states
        self.js_x = 0
        self.js_y = 0
        
        #IMU states
        self.imu_ts = 0;
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
        
        # msg between the main process and the CAN recv process
        self.vehicle_state = np.zeros(VEHICLE_STATE_N, dtype=np.float64)
        
        self.ctrl_state = np.zeros(CTRL_STAT_N, dtype=np.float64)
        
        self.flocking_init()
        
        
        # msg between the main process and the CAN send process 
        self.ctrl_command = np.zeros(COMMAND_N, dtype=np.float64)


        self.q_can_recv = q_can_recv
        self.q_can_recv_ctrl = q_can_recv_ctrl

        self.q_can_send = q_can_send
        self.q_can_send_ctrl = q_can_send_ctrl


        # V2V states

        # these are the most recent V2V records received from other vehicles
        self.v2v_records = np.zeros((NR_VEHICLES, V2V_N), np.float64)
        

        # V2I states

        # most recently V2I command (received) and state report (sent)
        self.v2i_command_msg = np.zeros(4, dtype=np.float64)
        self.v2i_vstate_msg = np.zeros(V2I_N, dtype=np.float64)

        
        pass


    def init_with_config_file(self):
        
        with open(CONFIG_FILE_NAME, mode='r') as fd:
        
            config_str = fd.read()
        
            config = json.loads(config_str)
            self.config = config
            
            self.vid = int(config['vid'])
            self.mode = config['mode']
            self.dbc_file = config['dbc_file']
            self.has_js = int(config['has_js'])
            
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
        ctrl_log_file = open('./log/ctrl_can.csv', 'w')

        while not self.stop:
        
            if not self.q_can_recv.empty():
        
                states_in_q = self.q_can_recv.get()
                
                self.vehicle_state = states_in_q[:VEHICLE_STATE_N]
                self.ctrl_state = states_in_q[VEHICLE_STATE_N:]
                
                ts = time.time()
                
                self.log_csv_record(chassis_log_file, ts, self.vehicle_state)
                self.log_csv_record(ctrl_log_file, ts, self.ctrl_state)
            
            
            
            time.sleep(0.01)
        
        self.q_can_recv_ctrl.put(np.zeros(1))
        
        print('chassis_can_recv_thread stopped.')


    def construct_ctrl_command(self, command):
        
        command[0] = self.vehicle_speed_dsr
        command[1] = self.vehicle_steer_dsr
        
        # control enable, sent by the control center
        command[2] = self.ctrl_enable
        
        # Emergency Brake, sent by the control center
        command[3] = self.emergency_brake
        
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
        
        command[12] = self.flocking_ctrl_u_vec[0]
        command[13] = self.flocking_ctrl_u_vec[1]
        
        
        command[14] = self.f_alpha[0]
        command[15] = self.f_alpha[1]
        command[16] = self.f_beta[0]
        command[17] = self.f_beta[1]
        command[18] = self.f_gamma[0]
        command[19] = self.f_gamma[1]
        
        
        
        # These are not sent over CAN but logged for debugging
        
        
        
        
        command[20:24] = self.self_vec
        command[24:28] = self.leader_vec
        command[28:32] = self.agent_vec[:, 0]
        command[32:36] = self.agent_vec[:, 1]
        command[36:40] = self.obstacle_vec[:, 0]
        
        
        
        
        
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


    def flocking_init(self):
        
        # flocking controller state
        self.flocking_ctrl_u_vec = np.zeros((2))
        
        self.self_vec = np.zeros((4))
        self.agent_vec = np.zeros((4, 2))
        self.obstacle_vec = np.zeros((4, 3))
        self.leader_vec = np.zeros((4))
        
        
        self.f_alpha = np.zeros((2))
        self.f_beta = np.zeros((2))
        self.f_gamma = np.zeros((2))


    def flocking_construct_self_vec(self, para):


        self.self_vec[0] = self.gps_baseline_pos_ned[2] # vel_x
        self.self_vec[1] = self.gps_baseline_pos_ned[1] # vel_y
        self.self_vec[2] = self.gps_baseline_vel_ned[2] # pos_x
        self.self_vec[3] = self.gps_baseline_vel_ned[1] # pos_y
        
        self.self_vec = self.self_vec / 1000


    def flocking_init_alpha_vec(self, para):
        
        nr_alpha = 2
        
        self.agent_vec = np.zeros((4, nr_alpha))
        
        self.agent_vec[0, 0] = 0.0
        self.agent_vec[1, 0] = 5.0
        self.agent_vec[2, 0] = 0.0
        self.agent_vec[3, 0] = 0.0

        self.agent_vec[0, 1] = 500.0
        self.agent_vec[1, 1] = 0.0
        self.agent_vec[2, 1] = 0.0
        self.agent_vec[3, 1] = 0.0


    def flocking_construct_agent_vec(self, para, last_ts, current_ts):
        '''Constructing states of agents which are close enough.
        
        
        '''
        
        agent_vec = self.agent_vec
        
        agent_j1 = ((self.vid - 1) + 1) % 3 + 1
        agent_j2 = ((self.vid - 1) + 2) % 3 + 1
        
        if self.vid == 1:
            
            agent_j1 = 2
            agent_j2 = 3
        
        if self.vid == 2:
            
            agent_j1 = 3
            agent_j2 = 1
        
        if self.vid == 3:
            
            agent_j1 = 1
            agent_j2 = 2
        
        
        
        agent_j1_v2v_record = self.v2v_records[agent_j1]
        agent_j2_v2v_record = self.v2v_records[agent_j2]
        
        
        agent_vec[0, 0] = agent_j1_v2v_record[5] # east is x
        agent_vec[1, 0] = agent_j1_v2v_record[4] # north is y
        agent_vec[2, 0] = agent_j1_v2v_record[7] # east is x
        agent_vec[3, 0] = agent_j1_v2v_record[6] # north is y
 
        agent_vec[0, 1] = agent_j2_v2v_record[5] # east is x
        agent_vec[1, 1] = agent_j2_v2v_record[4] # north is y
        agent_vec[2, 1] = agent_j2_v2v_record[7] # east is x
        agent_vec[3, 1] = agent_j2_v2v_record[6] # north is y
    
        
        
        delta_t = current_ts - last_ts
        
#         agent_vec[0, 0] += agent_vec[2, 0] * delta_t
#         agent_vec[1, 0] += agent_vec[3, 0] * delta_t
#             
#         agent_vec[2, 0] = self.js_x * 2
#         agent_vec[3, 0] = self.js_y * 2

        
        self.agent_vec = agent_vec


    def flocking_construct_obstacle_vec(self, para):
        '''Constructing states of obstacles which are close enough.
        
        obstacle_vec is a 4-by-n vector, (q_x, q_y, p_x, p_y)
        Here p_x and p_y are always zero.
        
        '''
        
        # assume currently there is only one obstacle
        nr_beta = 3
        
        # first obstacle, cylinder shape
        
        ob_pos = np.zeros((2))
        ob_pos[0] = para.ob_x
        ob_pos[1] = para.ob_y
        ob_r = para.ob_r
        
        self_pos = self.self_vec[0:2]
        
        u = self_pos - ob_pos
        un = np.linalg.norm(u)
        
        qk = ob_pos + u / un * ob_r
        
        
        #obstacle_vec = np.zeros((4, nr_beta))
        obstacle_vec = self.obstacle_vec

        obstacle_vec[0:2, 0] = qk
        
        
        # wall_1 and wall_2
        
        self_pos_x = self.self_vec[0]
        
        obstacle_vec[0, 1] = self_pos_x
        obstacle_vec[1, 1] = para.wall_1_y
        
        
        obstacle_vec[0, 2] = self_pos_x
        obstacle_vec[1, 2] = para.wall_2_y
        
        
        self.obstacle_vec = obstacle_vec


    def flocking_init_leader_vec(self, para):
        
        #self.leader_vec = np.zeros((4))
        self.leader_vec[0] = para.leader_x
        self.leader_vec[1] = para.leader_y
        self.leader_vec[2] = para.leader_vx
        self.leader_vec[3] = para.leader_vy


    def flocking_construct_leader_vec(self, para, 
        last_ts, current_ts):

        leader_vec = self.leader_vec
        
        if para.leader_mode == 'js':

            delta_t = current_ts - last_ts
            
            leader_vec[0] += leader_vec[2] * delta_t
            leader_vec[1] += leader_vec[3] * delta_t
            
            leader_vec[2] = self.js_x * 2
            leader_vec[3] = self.js_y * 2

        elif para.leader_mode == 'auto':

            delta_t = current_ts - last_ts
            
            if self.ctrl_enable > 0:
            
                leader_vec[0] += leader_vec[2] * delta_t
                leader_vec[1] += leader_vec[3] * delta_t

                if leader_vec[0] > 15:
                    leader_vec[2] = 0

                if leader_vec[1] > 9.5:
                    leader_vec[3] = 0

        elif para.leader_mode == 'v2v':
            
            leader_vid = para.leader_vid
            
            
            
            if leader_vid >= 0 and leader_vid < NR_VEHICLES:
            
                #print(self.v2v_records[leader_vid, 8:12])
            
                leader_vec[:] = self.v2v_records[leader_vid, 8:12]
                #leader_vec[0] -= 3
                leader_vec[1] += 3


        self.leader_vec = leader_vec


    def flocking_ctrl_calculate_alpha(self, para, self_vec, agent_vec):

        sigma = para.sigma
        
        r_alpha = sigma_norm_scalar(para.r_a, para.sigma)
        h_alpha = para.h_alpha
        d_alpha = sigma_norm_scalar(para.d_a, para.sigma)
        a_alpha = para.a_alpha
        b_alpha = para.b_alpha
        
        nr_agent = agent_vec.shape[1]
        
        f_alpha_1 = np.zeros((2))
        f_alpha_2 = np.zeros((2))
        
        for i in range(nr_agent):
            
            diff = agent_vec[:, i] - self_vec
            diff_pos = diff[0:2]
            diff_vel = diff[2:4]

            d_pos = np.linalg.norm(diff_pos)
            
            if d_pos > para.r_a:
                
                continue
            
            z_pos = sigma_norm(diff_pos, para.sigma)
            
            f_alpha_1_i = calculate_f_alpha_1_one_agent(\
                z_pos, diff_pos, r_alpha, h_alpha,
                d_alpha, sigma, a_alpha, b_alpha)

            f_alpha_2_i = calculate_f_alpha_2_one_agent(\
                z_pos, diff_vel, r_alpha, h_alpha)

            f_alpha_1 += f_alpha_1_i
            f_alpha_2 += f_alpha_2_i

        f_alpha = para.c1_alpha * f_alpha_1 + para.c2_alpha * f_alpha_2
    
        return f_alpha


    def flocking_ctrl_calculate_beta(self, para, self_vec, obstacle_vec):
    
        sigma = para.sigma
        
        r_beta = sigma_norm_scalar(para.r_b, para.sigma)
        h_beta = para.h_beta
        
        nr_obstacle = obstacle_vec.shape[1]
    
        f_beta_1 = np.zeros((2))
        f_beta_2 = np.zeros((2))
    
        for i in range(nr_obstacle):

            diff = obstacle_vec[:, i] - self_vec
            diff_pos = diff[0:2]
            diff_vel = diff[2:4]
            
            d_pos = np.linalg.norm(diff_pos)
            
            if d_pos > para.r_b:
                
                continue
            
            z_pos = sigma_norm(diff_pos, para.sigma)
    
            f_beta_1_i = calculate_f_beta_1_one_obstacle(\
                z_pos, diff_pos, r_beta, h_beta, sigma)

            f_beta_2_i = calculate_f_beta_2_one_obstacle(\
                z_pos, diff_vel, r_beta, h_beta)

            f_beta_1 += f_beta_1_i
            f_beta_2 += f_beta_2_i
        
        
        f_beta = para.c1_beta * f_beta_1 + para.c2_beta * f_beta_2
    
        return f_beta


    def flocking_ctrl_one_iteration(self):
        ''' Run the flocking controller for one iteration.
        
        
        self_vec is a 4 by 1 vector, (q_x, q_y, p_x, p_y)
            Here q_x and q_y are velocities in x and y in global frame,
            while p_x and p_y are positions in x and y in global frame.
        
        agent_vec is a 4-by-n vec and each column is similar to self_vec.
        
        '''
        
        para = self.para
        self_vec = self.self_vec
        agent_vec = self.agent_vec
        obstacle_vec = self.obstacle_vec
        leader_vec = self.leader_vec
        
        
        f_alpha = np.zeros((2))
        f_beta = np.zeros((2))
        f_gamma = np.zeros((2))
        
        u_vec = np.zeros((2))
        
        
        if agent_vec is not None:
            
            f_alpha = self.flocking_ctrl_calculate_alpha(para, self_vec, agent_vec)
        
        if obstacle_vec is not None:
            
            f_beta = self.flocking_ctrl_calculate_beta(para, self_vec, obstacle_vec)
        
        if leader_vec is not None:
            
            diff = leader_vec - self_vec
            diff_pos = diff[0:2]
            diff_vel = diff[2:4]
            
            f_gamma = para.c1_gamma * diff_pos + para.c2_gamma * diff_vel
        
        if self.ctrl_enable > 0:
            
            u_vec[:] = para.c_alpha * f_alpha \
                    + para.c_beta * f_beta \
                    + para.c_gamma * f_gamma
        
            
        
        self.self_vec = self_vec
        self.agent_vec = agent_vec
        self.obstacle_vec = obstacle_vec
        self.leader_vec = leader_vec
        
        
        self.f_alpha = f_alpha
        self.f_beta = f_beta
        self.f_gamma = f_gamma
        
        
        if self_vec[0] > 30 or self_vec[0] < -20:
            
            u_vec[:] = 0 

        if self_vec[1] > 12 or self_vec[1] < 0:
            
            u_vec[:] = 0
        
        
        self.flocking_ctrl_u_vec = u_vec


    def flocking_ctrl_thread(self):
    
        # CAUTION: it is needed to wait for a few seconds so that the sensors
        # are initialized and correct states are obtained.
        time.sleep(1)
    
        self.flocking_init_alpha_vec(self.para)
    
        self.flocking_init_leader_vec(self.para)
    
        last_ts = time.time()
    
        i = 0
        
        while not self.stop:
            
            current_ts = time.time()
            
            # obtain the state of the vehicle itself
            self.flocking_construct_self_vec(self.para)
            
            # construct agent_vec
            self.flocking_construct_agent_vec(
                self.para, last_ts, current_ts)
            
            
            # construct obstacle_vec
            self.flocking_construct_obstacle_vec(self.para)
            
            
            # construct leader vec
            
            self.flocking_construct_leader_vec(
                self.para, last_ts, current_ts)
    
    
            if i % 10 == 9:
                
                print('%d\t' % self.ctrl_enable, end='')
                
                for i in range(4):
                    
                    print('%4.1f, ' % self.self_vec[i], end='')

                print('  ', end='')

                for i in range(4):
                    
                    print('%4.1f, ' % self.leader_vec[i], end='')

                print('(%4.2f)' % self.para.c_gamma, end='')
                
                print('  ', end='')

                for i in range(4):
                    
                    print('%4.1f, ' % self.agent_vec[:, 0].flatten()[i], end='')
                
                print('    ', end='')
                
                for i in range(4):
                     
                    print('%4.1f, ' % self.agent_vec[:, 1].flatten()[i], end='')

                print('   (%4.2f)' % self.para.c_alpha, end='')
                
                print('    ', end='')
                
                print('%4.1f, %4.1f' % (self.obstacle_vec[1, 1], self.obstacle_vec[1, 2]), end='')
                
                print('    ', end='')
                
                print('  (%5.3f, %6.3f), ' \
                      % (self.flocking_ctrl_u_vec[0], 
                         self.flocking_ctrl_u_vec[1]), end='')

#                 print('(%6.3f, %6.3f)\t' \
#                       % (self.f_alpha[0], 
#                          self.f_alpha[1]), end='')

                print('(%6.3f, %6.3f)\t' \
                      % (self.f_alpha[0], 
                         self.f_alpha[1]), end='')

                print('(%6.3f, %6.3f)\t' \
                      % (self.f_beta[0], 
                         self.f_beta[1]), end='')
                 
                print()
    
            self.flocking_ctrl_one_iteration()
            
            
            last_ts = current_ts
            
            i += 1
            
            time.sleep(0.05)
    
        pass


    def v2v_send_thread(self):
        
        print('v2v_send_thread started')
    
        v2v_send_log = open('./log/v2v_send', 'w')
    
        
        dst_ips = ["192.168.1.11", "192.168.1.12", "192.168.1.13"]
        #dst_ips = ["192.168.137.1", "192.168.137.2", "192.168.137.3"]
        dst_port = 15000
        
        src_ip = "192.168.1.1%d" % self.vid
        #src_ip = "192.168.137.%d" % self.vid
        src_port = 15500 + self.vid
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
        sock.bind((src_ip, src_port))
    
        #msg_array = np.random.rand(17)
    
        
        msg_array = np.zeros(V2V_N, dtype=np.float64)
    
    
        while not self.stop:
            
            msg_array[0] = self.vid
            msg_array[1] = self.pos_llh[0]
            msg_array[2] = self.vehicle_speed_dsr
            msg_array[3] = self.vehicle_steer_dsr

            msg_array[4] = self.gps_baseline_pos_ned[1]
            msg_array[5] = self.gps_baseline_pos_ned[2]
            msg_array[6] = self.gps_baseline_vel_ned[1]
            msg_array[7] = self.gps_baseline_vel_ned[2]
            
            msg_array[4:8] = msg_array[4:8] / 1000

            msg_array[8:12] = self.leader_vec
            
            #msg_array[8:24] = self.imu_sample
            

            #msg_array[23] = self.gps_fixed_origin_pos_ned[0]
            #msg_array[24] = self.gps_fixed_origin_pos_ned[1]


            for dst_ip in dst_ips:

    
                sock.sendto(msg_array.tobytes(), (dst_ip, dst_port))
        
        
            v2v_csv = ','.join(['%f' % value for value in msg_array])
            
            v2v_send_log.write(v2v_csv)
            v2v_send_log.write('\n')
            
        
        
            time.sleep(0.05);
        
        pass


    def v2v_recv_thread(self):
        
        print('v2v_recv_thread started')
        
        v2v_recv_log = open('./log/v2v_recv', 'w')
        
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
            
            v2v_recv_log.write(v2v_csv)
            v2v_recv_log.write('\n')


    def v2i_send_thread(self):
        
        print('v2i_send_thread started')
    
        v2i_send_log = open('./log/v2i_send', 'w')
        
        dst_ip = "192.168.1.5"
        dst_port = 16000
        
        
        src_ip = "192.168.1.%d" % (self.vid + 10)
        src_port = 16500 + self.vid
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
        sock.bind((src_ip, src_port))
    
        #msg_array = np.random.rand(17)
    
        
        
        v2i_vstate_msg = self.v2i_vstate_msg
    
        while not self.stop:
            
            time.sleep(0.1);
            
            v2i_vstate_msg[0] = self.vid
            v2i_vstate_msg[1] = time.time()
            v2i_vstate_msg[2] = self.pos_llh[0]
            
            # chassis data
            
            chassis_columns = [0, 1, 2, 3, 4, 5, 9, 15, 21, 27, 31, 33]
            
            for i in range(12):
            
                col = chassis_columns[i]
                
                v2i_vstate_msg[i + 3] = self.vehicle_state[col]
            
            v2i_vstate_msg[15:31] = self.imu_sample
            v2i_vstate_msg[31:36] = self.gps_baseline_pos_ned[1:6]
            v2i_vstate_msg[36:41] = self.gps_baseline_vel_ned[1:6]
            v2i_vstate_msg[41:46] = self.pos_llh[1:6]
            
            
            try:
            
                sock.sendto(v2i_vstate_msg.tobytes(), (dst_ip, dst_port))


            except:
                
                continue
        
            v2i_csv = ','.join(['%f' % value for value in v2i_vstate_msg])
            
            v2i_send_log.write(v2i_csv)
            v2i_send_log.write('\n')
            
        
        pass


    def v2i_recv_thread(self):
        
        print('v2i_recv_thread started')
        
        v2i_recv_log = open('./log/v2i_recv', 'w')
        
        port = 17000
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
       
        sock.bind(('', port))
        
        sock.settimeout(1)

        while not self.stop:
            
            try:
            
                data, addr = sock.recvfrom(1024)
            
            except:
                
                continue
            
            
            v2i_command_msg = np.frombuffer(data, dtype=np.float64)
            
            v2i_vid = v2i_command_msg[0]
            
            
            self.exp_start = v2i_command_msg[1]
            self.ctrl_enable = v2i_command_msg[2]
            
            #print('---V2I---')
            
            
            v2i_csv = ','.join(['%f' % value for value in v2i_command_msg])
            
            v2i_recv_log.write(v2i_csv)
            v2i_recv_log.write('\n')


    def keyboard_thread(self):
        
        time.sleep(2)
        
        print('keyboard thread started.')
        
        while not self.stop:
            
            key = input('->')
            
            if key == 'exit':
                
                self.stop = True

            if key == 'enable':
                
                self.ctrl_enable = True
                
                print("ENABLE!!!")

            if key == 'disable':
                
                self.ctrl_enable = False
                
                print("DISABLE!!!")
                
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
                
                self.js_y = -jsev_value / 32768
            
            if jsev_number == 1:
                
                self.js_x = -jsev_value / 32768
            
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
                
                self.vehicle_speed_dsr = self.js_x * 150
                self.vehicle_steer_dsr = self.js_y * 150
                
            time.sleep(0.01);


    def main_loop(self):
        
        threads = []

        x_keyboard = threading.Thread(target=self.keyboard_thread)
        threads.append(x_keyboard)
        x_keyboard.start()
          
          
        if self.has_js > 0:
              
            x_js = threading.Thread(target=self.js_thread)
            threads.append(x_js)
            x_js.start()
          
  
        x_gps = threading.Thread(target=self.gps_thread)
        threads.append(x_gps)
        x_gps.start()
            
    
        x_sensor = threading.Thread(target=self.arduino_sensor_thread)
        threads.append(x_sensor)
        x_sensor.start()
    
        x_can_recv = threading.Thread(target=self.chassis_can_recv_thread)
        threads.append(x_can_recv)
        x_can_recv.start()
      
        x_can_send = threading.Thread(target=self.chassis_can_send_thread)
        threads.append(x_can_send)
        x_can_send.start()


        x_flocking_ctrl = threading.Thread(target=self.flocking_ctrl_thread)
        threads.append(x_flocking_ctrl)
        x_flocking_ctrl.start()



 
        x_v2v_send = threading.Thread(target=self.v2v_send_thread)
        threads.append(x_v2v_send)
        x_v2v_send.start()
     
        x_v2v_recv = threading.Thread(target=self.v2v_recv_thread)
        threads.append(x_v2v_recv)
        x_v2v_recv.start()


#         x_v2i_send = threading.Thread(target=self.v2i_send_thread)
#         threads.append(x_v2i_send)
#         x_v2i_send.start()
#      
        x_v2i_recv = threading.Thread(target=self.v2i_recv_thread)
        threads.append(x_v2i_recv)
        x_v2i_recv.start()

       
#         x_drive = threading.Thread(target=self.drive_by_wire)
#         threads.append(x_drive)
#         x_drive.start()
        
        
        while not self.stop:
            
            time.sleep(1)
            
        time.sleep(1)
        
        print('main exit')


def can_recv_process(q_recv, q_recv_ctrl):
    
    rx_agent = AGVCANRxAgent(q_recv, q_recv_ctrl)
    
    rx_agent.run()
    
    pass


def can_send_process(q_send, q_send_ctrl):

    tx_agent = AGVCANTxAgent(q_send, q_send_ctrl)
    
    tx_agent.run()

    pass


if __name__ == '__main__':
    
    
    q_can_recv = Queue()
    q_can_recv_ctrl = Queue()
    
    q_can_send = Queue()
    q_can_send_ctrl = Queue()
    
    p1 = Process(target=can_recv_process, args=(q_can_recv, q_can_recv_ctrl))
    
    p2 = Process(target=can_send_process, args=(q_can_send, q_can_send_ctrl))
    
    p1.start()
    p2.start()
    
    agv = AGVPi(q_can_recv, q_can_recv_ctrl, q_can_send, q_can_send_ctrl)
    
    agv.main_loop()
    
#     p1.terminate()
#     p2.terminate()
    
    pass



