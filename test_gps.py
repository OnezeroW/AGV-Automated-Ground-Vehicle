'''
Created on Nov 10, 2019

@author: priori
'''


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








def test_gps():

    print("testing GPS started.")
    
    log_file_pos = open('./log/gps_pos', 'w')
    log_file_vel = open('./log/gps_vel', 'w')

    parser = argparse.ArgumentParser(description = "Swift Navigation SBP Example.")
    parser.add_argument(
        "-p",
        "--port",
        default=['/dev/ttyUSB0'],
        nargs=1,
        help="specify the serial port to use."
    )
    args = parser.parse_args()
    
    with PySerialDriver(args.port[0], baud=115200) as gps_serial_driver:
        with Handler(Framer(gps_serial_driver.read, None, verbose=True)) as gps_handler:

#     gps_serial_driver = PySerialDriver(args.port[0], baud=115200)
#     
#     gps_handler = Handler(Framer(gps_serial_driver.read, None, verbose=True))
    
            print("GPS opened.")
          
            while True:
        
                    for msg, metadata in gps_handler.filter(SBP_MSG_POS_LLH):
                        current_gps_pos = msg
                        
                        log_file_pos.write(str(msg))
                        log_file_pos.write('\n')
                        log_file_pos.flush()
                        
                        print(msg, metadata)
                        break
                        
                    
        
            
                    for msg, metadata in gps_handler.filter(SBP_MSG_VEL_NED):
                        current_gps_vel = msg
                        
                        print("%.4f,%.4f,%.4f" % (msg.n * 1e-3, msg.e * 1e-3,
                                                      msg.d * 1e-3))
            
                        log_file_vel.write(str(msg))
                        log_file_vel.write('\n')
                        log_file_vel.flush()
            
                        print(msg, metadata)
                        break
                        
            
        
    
    pass




if __name__ == '__main__':
    
    
    test_gps()
    pass












