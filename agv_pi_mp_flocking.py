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


class AGVCANRxAgent(object):
    
    
    def __init__(self, q_can_recv, q_can_recv_ctrl):
        
        self.q_can_recv = q_can_recv
        self.q_can_recv_ctrl = q_can_recv_ctrl
        
        self.stop = False
        
        
        self.vehicle_state = np.zeros(34, dtype=np.float64)
        self.ctrl_state = np.zeros(24 + 20, dtype=np.float64)
        
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

        elif mid == 0x0A000200: # GPS time
            
            gps_time = struct.unpack('<ii', msg.data)
            
            self.ctrl_state[23] = gps_time[0] # GPS time

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
        
        
        # flocking other vehicle 1
        
        elif mid == 0x0A000300: # GPS time
            
            gps_time_ov1 = struct.unpack('<ii', msg.data)
            
            self.ctrl_state[24] = gps_time_ov1[0] # GPS time
        
        elif mid == 0x0A000303: # GPS velocity ENU
            
            gps_vel_ned_ov1 = struct.unpack('<ii', msg.data)
            
            self.ctrl_state[25] = gps_vel_ned_ov1[0] # velocity component in north
            self.ctrl_state[26] = gps_vel_ned_ov1[1] # velocity component in east

        elif mid == 0x0A000304: # GPS position ENU
            
            gps_pos_ned_ov1 = struct.unpack('<ii', msg.data)
            
            self.ctrl_state[27] = gps_pos_ned_ov1[0] # position component in north
            self.ctrl_state[28] = gps_pos_ned_ov1[1] # position component in east

        # flocking other vehicle 2
        
        elif mid == 0x0A000400: # GPS time
            
            gps_time_ov2 = struct.unpack('<ii', msg.data)
            
            self.ctrl_state[34] = gps_time_ov2[0] # GPS time
        
        elif mid == 0x0A000403: # GPS velocity ENU
            
            gps_vel_ned_ov2 = struct.unpack('<ii', msg.data)
            
            self.ctrl_state[35] = gps_vel_ned_ov2[0] # velocity component in north
            self.ctrl_state[36] = gps_vel_ned_ov2[1] # velocity component in east

        elif mid == 0x0A000404: # GPS position ENU
            
            gps_pos_ned_ov2 = struct.unpack('<ii', msg.data)
            
            self.ctrl_state[37] = gps_pos_ned_ov2[0] # position component in north
            self.ctrl_state[38] = gps_pos_ned_ov2[1] # position component in east
       
    
    # modified on 0527/2022
    def process_can_report_msg_using_dbc(self, msg):
        
        try:
            dmsg = self.can_dbc.decode_message(msg.arbitration_id, msg.data)
        except:
            pass

        #print('%X' % msg.arbitration_id)
        
        id = int(msg.arbitration_id)

        # vehicle state

        if msg.arbitration_id == 0x00000160:    # Steering_Feedback
            self.vehicle_state[0] = dmsg['Front_Str_Angle']
            self.vehicle_state[1] = dmsg['Front_Str_Spd']
            self.vehicle_state[2] = dmsg['Rear_Str_Angle']
            self.vehicle_state[3] = dmsg['Rear_Str_Spd']

        if msg.arbitration_id == 0x00000217:    # FL_Wheel_Spd_Feedback
            self.vehicle_state[4] = dmsg['FL_Wheel_Spd']

        if msg.arbitration_id == 0x00000227:    # FR_Wheel_Spd_Feedback
            self.vehicle_state[5] = dmsg['FR_Wheel_Spd']

        if msg.arbitration_id == 0x00000237:    # RL_Wheel_Spd_Feedback
            self.vehicle_state[6] = dmsg['RL_Wheel_Spd']

        if msg.arbitration_id == 0x00000247:    # RR_Wheel_Spd_Feedback
            self.vehicle_state[7] = dmsg['RR_Wheel_Spd']

        if msg.arbitration_id == 0x00000490:    # Brake_Feedback_1
            self.vehicle_state[8] = dmsg['Brake_Feedback_1']

        if msg.arbitration_id == 0x0B520101:    # Brake_Feedback_2
            self.vehicle_state[9] = dmsg['Brake_Feedback_2']

        if msg.arbitration_id == 0x1801FFF4:    # Battery_BMS
            self.vehicle_state[10] = dmsg['Current']
            self.vehicle_state[11] = dmsg['Voltage']
            self.vehicle_state[12] = dmsg['SOC']
            self.vehicle_state[13] = dmsg['Status_1']
            self.vehicle_state[14] = dmsg['Status_2']

        

        # control
        if msg.arbitration_id == 0x0A000200:    # GPS_Time
            self.ctrl_state[0] = dmsg['GPS_Time']

        if msg.arbitration_id == 0x0A000201:    # GPS_Lati
            self.ctrl_state[1] = dmsg['Lati_h']
            self.ctrl_state[2] = dmsg['Lati_l']

        if msg.arbitration_id == 0x0A000202:    # GPS_Long
            self.ctrl_state[3] = dmsg['Long_h']
            self.ctrl_state[4] = dmsg['Long_l']

        if msg.arbitration_id == 0x0A000203:    # GPS_Vel
            self.ctrl_state[5] = dmsg['GPS_VelN']
            self.ctrl_state[6] = dmsg['GPS_VelE']

        if msg.arbitration_id == 0x00000151:    # Steering_Brake_Control
            self.ctrl_state[7] = dmsg['Brake']
            self.ctrl_state[8] = dmsg['Front_Steering']
            self.ctrl_state[9] = dmsg['Rear_Steering']

        if msg.arbitration_id == 0x00000150:    # Wheel_Torque_Control
            self.ctrl_state[10] = dmsg['FL_Torque_Ctrl']
            self.ctrl_state[11] = dmsg['FR_Torque_Ctrl']
            self.ctrl_state[12] = dmsg['RL_Torque_Ctrl']
            self.ctrl_state[13] = dmsg['RR_Torque_Ctrl']

        pass
    
    
    
    
    def run(self):
        
        
        print('AGV CAN Rx Agent started.')
        
        chassis_log_file_raw = open('./log/chassis_can.raw', 'w')


        self.can_bus = can.interface.Bus(channel='can0', bustype='socketcan_native')
        self.can_dbc = cantools.database.load_file(self.dbc_file)
        
        ts_last = time.time()
        
        while not self.stop:
            
            msg = self.can_bus.recv()
            
            # self.process_can_report_msg(msg)
            self.process_can_report_msg_using_dbc(msg)

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
        
        self.ctrl_command = np.zeros(40, dtype=np.float64)
        
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

        
        
        # flocking, first vehicle
        
        gps_time_ov1 = command[20]
        
        gps_vn_ov1 = command[25]
        gps_ve_ov1 = command[26]

        gps_pn_ov1 = command[23]
        gps_pe_ov1 = command[24]
        
        gps_time_ov1_msg_struct = self.can_dbc.get_message_by_name('GPS_Time_OV1')
        gps_time_ov1_msg_body = gps_time_ov1_msg_struct.encode( \
            {'GPS_Time_OV1': gps_time_ov1})
        
        gps_time_ov1_msg = can.Message(arbitration_id = gps_time_ov1_msg_struct.frame_id, 
                                          data=gps_time_ov1_msg_body)
        self.can_bus.send(gps_time_ov1_msg)
        
        
        # GPS vn / ve
        
        gps_vel_ov1_msg_struct = self.can_dbc.get_message_by_name('GPS_Vel_OV1')
        gps_vel_ov1_msg_body = gps_vel_ov1_msg_struct.encode( \
            {'GPS_VelN_OV1': gps_vn_ov1,\
             'GPS_VelE_OV1': gps_ve_ov1})
        
        gps_vel_ov1_msg = can.Message(arbitration_id = gps_vel_ov1_msg_struct.frame_id, 
                                          data=gps_vel_ov1_msg_body)
        self.can_bus.send(gps_vel_ov1_msg)


        # GPS pn / pe

        gps_pos_ov1_msg_struct = self.can_dbc.get_message_by_name('GPS_Pos_OV1')
        gps_pos_ov1_msg_body = gps_pos_ov1_msg_struct.encode( \
            {'GPS_PosN_OV1': gps_pn_ov1,\
             'GPS_PosE_OV1': gps_pe_ov1})
        
        gps_pos_ov1_msg = can.Message(arbitration_id = gps_pos_ov1_msg_struct.frame_id, 
                                          data=gps_pos_ov1_msg_body)
        self.can_bus.send(gps_pos_ov1_msg)
        
        
        
        # flocking, first vehicle
        
        gps_time_ov2 = command[30]
        
        gps_vn_ov2 = command[35]
        gps_ve_ov2 = command[36]

        gps_pn_ov2 = command[33]
        gps_pe_ov2 = command[34]
        
        gps_time_ov2_msg_struct = self.can_dbc.get_message_by_name('GPS_Time_OV2')
        gps_time_ov2_msg_body = gps_time_ov2_msg_struct.encode( \
            {'GPS_Time_OV2': gps_time_ov2})
        
        gps_time_ov2_msg = can.Message(arbitration_id = gps_time_ov2_msg_struct.frame_id, 
                                          data=gps_time_ov2_msg_body)
        self.can_bus.send(gps_time_ov2_msg)
        
        
        # GPS vn / ve
        
        gps_vel_ov2_msg_struct = self.can_dbc.get_message_by_name('GPS_Vel_OV2')
        gps_vel_ov2_msg_body = gps_vel_ov2_msg_struct.encode( \
            {'GPS_VelN_OV2': gps_vn_ov2,\
             'GPS_VelE_OV2': gps_ve_ov2})
        
        gps_vel_ov2_msg = can.Message(arbitration_id = gps_vel_ov2_msg_struct.frame_id, 
                                          data=gps_vel_ov2_msg_body)
        self.can_bus.send(gps_vel_ov2_msg)


        # GPS pn / pe

        gps_pos_ov2_msg_struct = self.can_dbc.get_message_by_name('GPS_Pos_OV2')
        gps_pos_ov2_msg_body = gps_pos_ov2_msg_struct.encode( \
            {'GPS_PosN_OV2': gps_pn_ov2,\
             'GPS_PosE_OV2': gps_pe_ov2})
        
        gps_pos_ov2_msg = can.Message(arbitration_id = gps_pos_ov2_msg_struct.frame_id, 
                                          data=gps_pos_ov2_msg_body)
        self.can_bus.send(gps_pos_ov2_msg)
        
    
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
        self.imu_ts = 0;
        self.imu_sample = np.zeros(16, dtype=np.float64)

        # GPS states
        self.gps_imu_raw = (0,0,0,0,0,0,0,0)    # added on 0527/2022
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

        log_gps_imu_raw = open('./log/gps_imu_raw.csv', 'w')    # added on 0527/2022
    
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

                # added on 0527/2022
                if msg_type == 0x0900: # MSG_IMU_RAW
                    gps_imu_raw = struct.unpack('<IBhhhhhhH', payload)
                    self.gps_imu_raw = gps_imu_raw
                    self.log_csv_record(log_gps_imu_raw, ts, gps_imu_raw)
    
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
                
                self.vehicle_state = states_in_q[0:34]
                self.ctrl_state = states_in_q[34:]
                
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
        
    
    def v2v_send_thread(self):
        
        print('v2v_send_thread started')
    
        v2v_send_log = open('./log/v2v_send', 'w')
    
        
        #dst_ips = ["192.168.0.100", "192.168.0.11", "192.168.0.12", "192.168.0.13"]
        dst_ips = ["192.168.137.1", "192.168.137.2", "192.168.137.3"]
        dst_port = 15000
        
        #src_ip = "192.168.0.1%d" % self.vid
        src_ip = "192.168.137.%d" % self.vid
        src_port = 15500 + self.vid
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
        sock.bind((src_ip, src_port))
    
        #msg_array = np.random.rand(17)
    
        
        msg_array = np.zeros(V2V_N, dtype=np.float64)
    
    
        while not self.stop:
            
            msg_array[0] = self.vid
            msg_array[1] = time.time()
            msg_array[2] = self.vehicle_speed_dsr
            msg_array[3] = self.vehicle_steer_dsr
            
            msg_array[4:20] = self.imu_sample
            msg_array[20] = self.pos_llh[0]
            msg_array[21] = self.pos_llh[1]
            msg_array[22] = self.pos_llh[2]
            
            msg_array[23] = self.gps_baseline_pos_ned[1]
            msg_array[24] = self.gps_baseline_pos_ned[2]

            #msg_array[23] = self.gps_fixed_origin_pos_ned[0]
            #msg_array[24] = self.gps_fixed_origin_pos_ned[1]

            msg_array[25] = self.gps_baseline_vel_ned[1]
            msg_array[26] = self.gps_baseline_vel_ned[2]

              
                
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
        
        dst_ip = "192.168.1.4"
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
            
            
            v2i_csv = ','.join(['%f' % value for value in v2i_command_msg])
            
            v2i_recv_log.write(v2i_csv)
            v2i_recv_log.write('\n')
 

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

        # x_keyboard = threading.Thread(target=self.keyboard_thread)
        # threads.append(x_keyboard)
        # x_keyboard.start()
          
          
        # if self.mode == 'js':
              
        #     x_js = threading.Thread(target=self.js_thread)
        #     threads.append(x_js)
        #     x_js.start()
          
 
        x_gps = threading.Thread(target=self.gps_thread)
        threads.append(x_gps)
        x_gps.start()

        x_as = threading.Thread(target=self.arduino_sensor_thread)
        threads.append(x_as)
        x_as.start()
    
        x_vcan_recv = threading.Thread(target=self.chassis_can_recv_thread)
        threads.append(x_vcan_recv)
        x_vcan_recv.start()
      
        # x_vcan_send = threading.Thread(target=self.chassis_can_send_thread)
        # threads.append(x_vcan_send)
        # x_vcan_send.start()

 
        # x_v2v_send = threading.Thread(target=self.v2v_send_thread)
        # threads.append(x_v2v_send)
        # x_v2v_send.start()
     
        # x_v2v_recv = threading.Thread(target=self.v2v_recv_thread)
        # threads.append(x_v2v_recv)
        # x_v2v_recv.start()

        
        
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
