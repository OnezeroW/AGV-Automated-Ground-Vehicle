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
from multiprocessing import Process, Queue, log_to_stderr

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
        
        latitude = command[0]
        longitude = command[1]

        #latitude = 37.77405700243414
        #longitude = -122.41683960033353

        print('Lat and lon read successfully!')
        
        
        lat_h = int(latitude * 1e6)
        #lat_l = int((latitude * 1e6 - lat_h) * 1e8)
        lat_l = 12345678
    
        lon_h = int(longitude * 1e6)
        #lon_l = int((longitude * 1e6 - lon_h) * 1e8)
        lon_l = 87654321

        print(lat_h, lat_l, lon_h, lon_l)
        
        # GPS latitude / longitude
        
        gps_lat_msg_struct = self.can_dbc.get_message_by_name('GPS_Lati')
        gps_lat_msg_body = gps_lat_msg_struct.encode( \
            {'Lati_h': lat_h,\
             'Lati_l': lat_l})
        
        gps_lat_msg = can.Message(arbitration_id = gps_lat_msg_struct.frame_id, 
                                          data=gps_lat_msg_body)
        self.can_bus.send(gps_lat_msg)
        print('Lat sent!')
        

        gps_lon_msg_struct = self.can_dbc.get_message_by_name('GPS_Long')
        gps_lon_msg_body = gps_lon_msg_struct.encode( \
            {'Long_h': lon_h,\
             'Long_l': lon_l})
        
        gps_lon_msg = can.Message(arbitration_id = gps_lon_msg_struct.frame_id, 
                                          data=gps_lon_msg_body)
        self.can_bus.send(gps_lon_msg)
        print('Lon sent!')

        # add by Jialin on 05/11/2022
        # self.stop = True

    
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



    def main_loop(self):
        
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
    #     #q_can_send_ctrl.put(i)
    
    with open('test.csv') as file:
        test_data = np.loadtxt(file, delimiter=',')
    for i in range(101):
        lat = test_data[i][0]
        lon = test_data[i][1]
        q_can_send.put([lat,lon])
    
    #og_test = open('./log/test.csv', 'r')
   #for i in range(101):
    #   test_data = log_test.readline()
     #  lat = test_data[0]
      # lon = test_data[1]
       #q_can_send.put([lat,lon])
    
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

