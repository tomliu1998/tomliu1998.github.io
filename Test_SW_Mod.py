import argparse
import atexit
import logging
import math
import os
import sys
import time
import numpy as np

from cyber.python.cyber_py3 import cyber
from gflags import FLAGS

from modules.tools.common.logger import Logger
from modules.canbus.proto import chassis_pb2
from modules.localization.proto import localization_pb2
from numpy import genfromtxt
import scipy.signal as signal

from cyber.python.cyber_py3 import cyber_time
from modules.common.configs.proto import vehicle_config_pb2
from modules.common.proto import drive_state_pb2
from modules.common.proto import pnc_point_pb2
from modules.control.proto import pad_msg_pb2
from modules.planning.proto import planning_pb2
import modules.tools.common.proto_utils as proto_utils



class Test_SW_Mod():




    def __init__(self, node):
        self.chassis = chassis_pb2.Chassis()
        self.planning_pub = node.create_writer('/apollo/planning',
                                               planning_pb2.ADCTrajectory)
        self.planningdata = planning_pb2.ADCTrajectory()
        self.logger = Logger.get_logger(tag="RtkPlayer")




    def Standby_PD(self):
        self.publish_planningmsg()
        if self.chassis.control_mod ==4:
            if self.chassis.gear_location == 2 and self.chassis.parking_st == 2:
                self.planningdata.auto_pilot_req = 1
                self.planning_pub.write(self.planningdata)
            else:
                self.planningdata.parking_req = 2
                self.planning_pub.write(self.planningdata)
                self.planningdata.brk_pedl_posn_req = 100
                self.planning_pub.write(self.planningdata)
                self.planningdata.shift_pos_req = 2
                self.planning_pub.write(self.planningdata)
                if self.chassis.gear_location == 2 and self.chassis.parking_st == 2:
                    self.planningdata.auto_pilot_req = 1
                    self.planning_pub.write(self.planningdata)
                else:
                    self.logger.error("not in parking and N_Gear")
        else:
            self.logger.error("being not standby")
    
    def Parking_rels(self):
        self.publish_planningmsg()
        if self.chassis.control_mod == 1:
            self.planningdata.parking_req = 1
            self.planningdata.tgt_long_ctrl_mode = 1
            self.planningdata.auto_pilot_req = 0
            self.planning_pub.write(self.planningdata)
            self.planningdata.brk_pedl_posn_req = 100
            self.planning_pub.write(self.planningdata)
            self.planningdata.shift_pos_req = 3
            self.planning_pub.write(self.planningdata)
            if self.chassis.gear_location == 3:
                self.planningdata.brk_pedl_posn_req = 0
                self.planning_pub.write(self.planningdata)
            else:
                self.logger.error("Gear_location is not D")
        else:
            self.logger.error("control mod is not AD")

    def AD_safe(self):
        if(self.chassis.control_mod == 1 and self.chassis.parking_st == 0 and self.chassis.gear_location == 3):
            self.Safe_PD = 1
            self.planningdata.parking_req = 0
            self.planning_pub.write(self.planningdata)
        else:
            self.Safe_PD = 0
            self.logger.error("being not safe for AD")
        return self.Safe_PD





    def SW_mode(self):
        self.publish_planningmsg()
        self.planningdata.tgt_steer_mode_req = 5
        self.planningdata.tgt_long_speed = 0
        self.planningdata.tgt_steer_angle_req = 0
        self.planningdata.shift_pos_req = 2
        self.planningdata.brk_pedl_posn_req = 0
        self.planning_pub.write(self.planningdata)



    def chassis_callback(self, data):
        """
        New chassis Received
        """
        self.chassis.CopyFrom(data)
        self.automode = True
        self.chassis_received = True



    def publish_planningmsg(self):
        #self.planningdata = planning_pb2.ADCTrajectory()
        self.planningdata.header.module_name = "planning"
        
        self.planningdata.tgt_steer_mode_req = 5
        self.planningdata.tgt_long_speed = 1
        self.planningdata.tgt_steer_angle_req = 0
        self.planningdata.shift_pos_req = 3


    def main():
        
        node = cyber.Node("Test_SW_Mod")       
        
        Switcher = Test_SW_Mod(node)


        node.create_reader('/apollo/canbus/chassis', chassis_pb2.Chassis,
                       player.chassis_callback)





        Switcher.Standby_PD()
        #time.sleep(2)
        Switcher.Parking_rels()
        #time.sleep(2)
        Switcher.AD_safe()
        #time.sleep(2)

        Switcher.SW_mode()
        time.sleep(10)

        while(self.chassis.control_mod == 1 and self.chassis.parking_st == 0 and self.chassis.gear_location == 3):
            Switcher.publish_planningmsg()

