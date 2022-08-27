#!/usr/bin/env python3

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################
"""
Record GPS and IMU data
"""

import argparse
import atexit
from cmath import pi
import logging
import math
import os
import sys
import time
from tokenize import Double
import numpy as np

from cyber.python.cyber_py3 import cyber
from gflags import FLAGS

from modules.tools.common.logger import Logger
from modules.canbus.proto import chassis_pb2
from modules.canbus.proto import chassis_detail_pb2
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

# TODO(all): hard-coded path temporarily. Better approach needed.
APOLLO_ROOT = "/apollo"

SEARCH_INTERVAL = 100
SEARCH_CUR = 1000
CHANGE_TO_COM = False
SPEED_PLAN_STEP = 200
CLOSE_POINT_STEP = 100

class RtkPlayer(object):
    """
    rtk player class
    """

    def __init__(self, record_file, node, speedmultiplier, completepath,
                 replan):
        """Init player."""
        self.firstvalid = False
        self.logger = Logger.get_logger(tag="RtkPlayer")
        self.logger.info("Load record file from: %s" % record_file)
        try:
            file_handler = open(record_file, 'r')# 'r':open for reading (default)
        except (FileNotFoundError, IOError) as ex:
            self.logger.error("Error opening {}: {}".format(record_file, ex))
            sys.exit(1)

        self.data = genfromtxt(file_handler, delimiter=',', names=True)
        file_handler.close()

        self.localization = localization_pb2.LocalizationEstimate()
        self.chassis = chassis_pb2.Chassis()
        self.chassis_detail = chassis_detail_pb2.ChassisDetail()
        self.padmsg = pad_msg_pb2.PadMessage()
        self.localization_received = True
        self.chassis_received = False

        self.planning_pub = node.create_writer('/apollo/planning',
                                               planning_pb2.ADCTrajectory)

        self.speedmultiplier = speedmultiplier / 100
        self.terminating = False
        self.sequence_num = 0


        self.start = 0
        self.end = 0
        self.closestpoint = 0
        self.automode = False

        self.replan = (replan == 't')
        self.completepath = (completepath == 't')

        self.planningdata = planning_pb2.ADCTrajectory()

        self.estop = False
        self.logger.info("Planning Ready")


        #输入汽车的参数(默认林肯MKZ)
        vehicle_config = vehicle_config_pb2.VehicleConfig()
        proto_utils.get_pb_from_text_file(
            "/apollo/modules/common/data/vehicle_param.pb.txt", vehicle_config)
        self.vehicle_param = vehicle_config.vehicle_param

    def localization_callback(self, data):
        """
        New localization Received
        """
        self.localization.CopyFrom(data)
        self.carx = self.localization.pose.position.x
        self.cary = self.localization.pose.position.y
        self.carz = self.localization.pose.position.z
        self.cartheta = self.localization.pose.heading
        self.localization_received = True


    def chassis_callback(self, data):
        """
        New chassis Received
        """
        self.chassis.CopyFrom(data)
        self.automode = True
        self.chassis_received = True
        
    def chassis_detail_callback(self, data):
        """
        New chassis Received
        """
        self.chassis_detail.CopyFrom(data)
        self.automode = True
        
    def padmsg_callback(self, data):
        """
        New message received
        """
        if self.terminating is True:
            self.logger.info("terminating when receive padmsg")
            return

        self.padmsg.CopyFrom(data)


    def lateral_dist(self, d_x, d_y):
        shortest_dist_sqr = 100000.0
        self.carx = self.localization.pose.position.x
        self.cary = self.localization.pose.position.y
        shortest_dist_sqr = float('inf')
        self.logger.info("before closest self.start=%s" % (self.start))
        search_start = max(self.start - SEARCH_INTERVAL // 2, 0)
        search_end = min(self.start + SEARCH_INTERVAL // 2, len(self.data))
        self.logger.debug("search_start: %s" % search_start)
        self.logger.debug("search_end: %s" % search_end)
        self.logger.debug("self.start: %s" % self.start)
        for j in range(search_start, search_end):
            dist_sqr = (self.carx - d_x[j]) ** 2 + \
                (self.cary - d_y[j]) ** 2
            search_start += 1
            search_end = min(self.start + SEARCH_INTERVAL // 2, len(self.data))

            S_xy = (d_x[j]-self.carx)*(d_y[j+1]-self.cary)-(d_y[j]-self.cary)*(d_x[j+1]-self.carx)
            if S_xy > 0:
                dir_one = 1
            elif S_xy < 0:
                dir_one = -1
            else:
                dir_one = 0
            if dist_sqr <= shortest_dist_sqr:       
                shortest_dist_sqr = dist_sqr
                lateral_dist_sqrt = math.sqrt(shortest_dist_sqr)*dir_one
        
        # failed to find a trajectory matches current gear position
        if shortest_dist_sqr == 100000.0:
            self.logger.info(
                'can not find closet point')
        #有关水平偏差的正负值以及单位(m)还需确认
        return lateral_dist_sqrt
    
    def closet_point(self, k_cp):
        shortest_dist_unsqrt = 1000000.0
        #self.car_x = self.localization.pose.position.x
        #self.car_y = self.localization.pose.position.y
        for i_mode in range(k_cp, (k_cp+self.i_js_4-1)):
            closet_dist_unsqrt = (self.carx - self.data['x'][i_mode])**2 + (self.cary - self.data['y'][i_mode])**2
            S_close_xy = (self.data['x'][i_mode]-self.carx)*(self.data['y'][i_mode+1]-self.cary)-(self.data['y'][i_mode]-self.cary)*(self.data['x'][i_mode+1]-self.carx)

            if S_close_xy > 0:
                closet_one = 1
            elif S_close_xy < 0:
                closet_one = -1
            else:
                closet_one = 0
            
            if closet_dist_unsqrt <= shortest_dist_unsqrt:
                shortest_dist_unsqrt = closet_dist_unsqrt
                i_cp = i_mode
                closet_dist = math.sqrt(shortest_dist_unsqrt)*closet_one
        if shortest_dist_unsqrt == 1000000.0:
            self.logger.error("can not find closet_dist")
        
        return i_cp, closet_dist
            





    def cal_cur(self, i_cur):
        fit_xy =  np.polyfit(self.data['y'][i_cur:i_cur+SEARCH_CUR], self.data['x'][i_cur:i_cur+SEARCH_CUR], 2)
        fit_xy_a = (1+(2*fit_xy[0]*self.data['y'][i_cur]+fit_xy[1])**2)**1.5
        fit_xy_b = np.absolute(2*fit_xy[0])
        if fit_xy_b == 0.0:
            fit_xy_b = 0.0001
        fit_xy_cur = fit_xy_a/fit_xy_b
        if self.data['WheelAngle_f'][i_cur] > 0:
            cur_one = 1
        elif self.data['WheelAngle_f'][i_cur] < 0:
            cur_one = -1
        else:
            cur_one = 0
        fit_cur_r = fit_xy_cur*cur_one
        return fit_cur_r




    def speed_planning(self, i_speed):
        t_planning = self.data['time'][i_speed+SPEED_PLAN_STEP] - self.data['time'][i_speed]
        S_planning_unsqrt = (self.data['x'][i_speed+SPEED_PLAN_STEP] - self.data['x'][i_speed])**2 + (self.data['y'][i_speed+SPEED_PLAN_STEP] - self.data['y'][i_speed])**2
        S_planning = math.sqrt(S_planning_unsqrt)
        speed_planning = (S_planning/t_planning)*3.6
        return speed_planning


    def cal_theta(self, i_theta):
        #self.cartheta = self.localization.pose.heading
        cal_m = (self.cartheta - self.data['theta'][i_theta])
        if  cal_m > math.pi:
            car_theta = (cal_m - 2*math.pi) * 180/math.pi
        elif cal_m < -math.pi:
            car_theta = (cal_m + 2*math.pi) * 180/math.pi
        else:
            car_theta = cal_m * 180/pi
        return car_theta






            
    def Standby_PD(self):
        #self.publish_planningmsg()
        if self.chassis_detail.schaeffler.ccu_status_1_208.ccu_ctrlmode == 4:
            if self.chassis_detail.schaeffler.ccu_status_1_208.ccu_parkingst == 2:
                self.planningdata.auto_pilot_req = 1
                self.planning_pub.write(self.planningdata)
            else:
                self.planningdata.brk_pedl_posn_req = 40
                self.planning_pub.write(self.planningdata)
                self.planningdata.parking_req = 2
                self.planning_pub.write(self.planningdata)
                self.planningdata.shift_pos_req = 2
                self.planning_pub.write(self.planningdata)
                if self.chassis_detail.schaeffler.ccu_status_1_208.ccu_parkingst == 2:
                    self.planningdata.auto_pilot_req = 1
                    self.planning_pub.write(self.planningdata)
                else:
                    self.logger.error("not in parking and N_Gear")
        else:
            self.logger.error("being not standby")
    
    def Parking_rels(self):
        #self.publish_planningmsg()
        if self.chassis_detail.schaeffler.ccu_status_1_208.ccu_ctrlmode == 1:
            self.planningdata.parking_req = 1
            self.planningdata.tgt_long_ctrl_mode = 1
            #self.planningdata.auto_pilot_req = 0
            self.planning_pub.write(self.planningdata)
            time.sleep(5)
            if self.chassis_detail.schaeffler.ccu_status_1_208.ccu_gearposst == 2:
                self.planningdata.brk_pedl_posn_req = 40
                self.planning_pub.write(self.planningdata)
                time.sleep(5)
                self.planningdata.shift_pos_req = 3
                self.planning_pub.write(self.planningdata)
                time.sleep(5)
                if self.chassis_detail.schaeffler.ccu_status_1_208.ccu_gearposst == 3:
                    self.planningdata.brk_pedl_posn_req = 0
                    self.planning_pub.write(self.planningdata)
                    time.sleep(5)
                else:
                    self.logger.error("Gear_location is not D")
            else:
                self.logger.error("Gear_location is not N")
        else:
            self.logger.error("control mod is not AD")

    def AD_safe(self):
        if(self.chassis_detail.schaeffler.ccu_status_1_208.ccu_ctrlmode == 1 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_parkingst == 0 \
            and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actbrkpedlposn == 0.0):
            self.Safe_PD = 1
            self.planningdata.parking_req = 0
            self.planningdata.tgt_steer_mode_req = 2
            self.planning_pub.write(self.planningdata)
            time.sleep(5)
        else:
            self.Safe_PD = 0
            self.logger.error("being not safe for AD")
        return self.Safe_PD





    def publish_planningmsg(self):
        """
        Generate New Path
        """
        self.logger.info("enter: publish_planningmsg ")
        if not self.localization_received:
            self.logger.warning(
                "localization not received yet when publish_planningmsg")
            return
  
        #self.planningdata = planning_pb2.ADCTrajectory()
        self.planningdata.header.module_name = "planning"
        self.planningdata.header.sequence_num = self.sequence_num
        self.sequence_num = self.sequence_num + 1
        #distpoint = self.closest_dist()
        self.end = len(self.data) - 1
        #fit_xy =  np.polyfit(self.data['y'], self.data['x'], 2)

        for i in range(self.start, self.end):
            self.logger.info("enter: publish_planningmsg: for i in rang ")
            self.i_js_4 = 0
            self.i_js_5 = 0

##############################################################################################################################
            # if self.data['SteerMode'][i] == 1 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode != 4:
            #     self.logger.warning("进入第 一 判断")
            #     self.planningdata.brk_pedl_posn_req = 40
            #     self.planning_pub.write(self.planningdata)
            #     time.sleep(5)

            #     while(self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode != 4):
            #         self.logger.warning("进入第一判断 模式切换循环 ")
            #         self.planningdata.tgt_steer_mode_req = 2
            #         self.planningdata.brk_pedl_posn_req = 0
            #         self.planning_pub.write(self.planningdata)
            #     #time.sleep(5)

            #     # self.u_4 = i

            #     # while(self.data['SteerMode'][self.u_4] == 4):
            #     #     self.i_js_4 += 1
            #     #     self.u_4 += 1

            #     if self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 4 and self.chassis_detail.schaeffler.ccu_status_1_208.has_ccu_strmodetransstate == 1:
            #         FourWheel_AD_safe = 1
            #     else:
            #         FourWheel_AD_safe = 0
            #         self.logger.error("FourWheel mode is not finished or steermode is not 4")
            #     self.logger.warning("FourWheel_AD_safe: %s", FourWheel_AD_safe)
        


            #if FourWheel_AD_safe == 1 or (self.data['SteerMode'][i] == 1 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 4 and self.data['gear'][i] == 3):
            if self.data['SteerMode'][i] == 1 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 4 and self.data['gear'][i] == 3:
                
                self.logger.info("enter: the second judge")
                
                self.u_4 = i
                self.i_first = float('inf')
                
                while(self.data['SteerMode'][self.u_4] == 1):
                    self.logger.info("enter: the second judge: counter ")
                    self.i_js_4 += 1
                    self.u_4 += 1
                    if self.u_4 <= self.i_first:
                        self.i_first = self.u_4
                    if self.u_4 == self.end-3:
                        break
                        
                if self.i_first == float('inf'):
                    self.logger.error("can not find i_first")
                
                self.logger.info("i_first: %s", self.i_first)


            #while(FourWheel_AD_safe == 1 or (self.data['SteerMode'][i] == 1 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 4 and self.data['gear'][i] == 3)):
            while(self.data['SteerMode'][i] == 1 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 4 and self.data['gear'][i] == 3):

                self.logger.info("enter: fourwheelmodel: main_while ")

                self.planningdata.tgt_steer_mode_req = 2
                i_now, self.planningdata.lateral_dis_deviation  = self.closet_point(self.i_first)
                #self.planningdata.lateral_dis_deviation  = 0
                self.planningdata.tgt_route_curvature = self.cal_cur(i_now)
                self.planningdata.headingangle_deviation = self.cal_theta(i_now)
                #self.planningdata.headingangle_deviation = 0
                self.planningdata.tgt_long_speed = self.speed_planning(i_now)
                #self.planningdata.tgt_long_speed = 3.0
                #self.planningdata.tgt_long_speed = 1.0
                self.planningdata.shift_pos_req = 3

                self.logger.info("lateral dis_deviation: %s" % self.planningdata.lateral_dis_deviation)
                self.logger.info("route curvate: %s" % self.planningdata.tgt_route_curvature)
                self.logger.info("planning speed: %s" % self.planningdata.tgt_long_speed)
                self.logger.info("act steer mode: %s" % self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode)
                self.logger.info("data_SteerMode: %s" % self.data['SteerMode'][i])
                self.logger.info("data_gear: %s" % self.data['gear'][i])
                
                self.planning_pub.write(self.planningdata)
                #time.sleep(0.1)
                #self.logger.debug("Generated Planning Sequence: "
                            #+ str(self.sequence_num - 1))
                i = i_now
                if (self.i_first + (self.i_js_4-1)-10) <= i_now <= (self.i_first + (self.i_js_4-1)):
                    i = self.i_first + (self.i_js_4-1)
                    break
                # time.sleep(0.05)

##############################################################################################################################
            # if self.data['SteerMode'][i] == 5 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode != 5:
            #     self.logger.info("enter: the Tr_mode first judge")
            #     self.planningdata.brk_pedl_posn_req = 40
            #     self.planning_pub.write(self.planningdata)
            #     time.sleep(5)

            #     while(self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode != 5):
            #         self.logger.info("enter: Tr_mode switch while ")
            #         self.planningdata.tgt_steer_mode_req = 5
            #         self.planningdata.tgt_long_speed = 0
            #         self.planningdata.tgt_steer_angle_req = 0
            #         self.planningdata.shift_pos_req = 2
            #         self.planningdata.brk_pedl_posn_req = 0
            #         self.planning_pub.write(self.planningdata)
            #     #time.sleep(10)

            #     # self.u_5 = i

            #     # while(self.data['SteerMode'][self.u_5] == 5):
            #     #     self.i_js_5 += 1
            #     #     self.u_5 += 1


            #     if self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 5 and self.chassis_detail.schaeffler.ccu_status_1_208.has_ccu_strmodetransstate == 1:
            #         Tr_AD_safe = 1
            #     else:
            #         Tr_AD_safe = 0
            #         self.logger.error("turning circle is not finished or steermode is not 5")
            #     self.logger.warning("Tr_AD_safe: %s" % Tr_AD_safe)
            
            # while(Tr_AD_safe == 1):
            #     self.logger.info("enter: Tr_mode main while")

            #     self.u_5 = i

            #     while(self.data['SteerMode'][self.u_5] == 5):
            #         self.logger.info("enter: Tr_mode counter while")
            #         self.i_js_5 += 1
            #         self.u_5 += 1

            #     self.planningdata.tgt_steer_mode_req = 5
            #     self.planningdata.tgt_long_speed = 1
            #     self.planningdata.tgt_steer_angle_req = 0
            #     self.planningdata.shift_pos_req = 3
            #     self.planning_pub.write(self.planningdata)
            #     if self.localization.pose.heading == self.data['theta'][(i+self.i_js_5-1)]:
            #         self.logger.info("enter: Tr_mode angle judge")
            #         self.planningdata.brk_pedl_posn_req = 40
            #         self.planning_pub.write(self.planningdata)
            #         time.sleep(2)
            #         i = i+self.i_js_5-1
            #         Tr_AD_safe = 0
            #         break
                    
            
            

            # while(self.data['SteerMode'][i] == 5 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 5):
            #     self.logger.info("enter: Tr_mode remain judge")
            #     i += 1
            #     if self.data['SteerMode'][i] != 5:
            #         break
        
        

            








    def shutdown(self):
        """
        shutdown cyber
        """
        self.terminating = True
        self.logger.info("Shutting Down...")
        time.sleep(0.2)

    def quit(self, signum, frame):
        """
        shutdown the keypress thread
        """
        sys.exit(0)

def main():
    """
    Main cyber
    """
    parser = argparse.ArgumentParser(
        description='Generate Planning Trajectory from Data File')
    parser.add_argument(
        '-s',
        '--speedmulti',
        help='Speed multiplier in percentage (Default is 100) ',
        type=float,
        default='100')
    parser.add_argument(
        '-c', '--complete', help='Generate complete path (t/F)', default='F')
    parser.add_argument(
        '-r',
        '--replan',
        help='Always replan based on current position(t/F)',
        default='F')
    args = vars(parser.parse_args())

    node = cyber.Node("rtk_player")

    Logger.config(
        log_file=os.path.join(APOLLO_ROOT, 'data/log/rtk_player.log'),
        use_stdout=True,
        log_level=logging.DEBUG)


    

    record_file = os.path.join(APOLLO_ROOT, 'data/log/garage.csv')

    player = RtkPlayer(record_file, node, args['speedmulti'],
                       args['complete'].lower(), args['replan'].lower())
    atexit.register(player.shutdown)

    node.create_reader('/apollo/canbus/chassis', chassis_pb2.Chassis,
                       player.chassis_callback)

    node.create_reader('/apollo/canbus/chassis_detail', chassis_detail_pb2.ChassisDetail,
                       player.chassis_detail_callback)

    node.create_reader('/apollo/localization/pose',
                       localization_pb2.LocalizationEstimate,
                       player.localization_callback)

    node.create_reader('/apollo/control/pad', pad_msg_pb2.PadMessage,
                       player.padmsg_callback)

    time.sleep(5)
    player.Standby_PD()
    time.sleep(5)
    player.Parking_rels()
    time.sleep(5)
    player.AD_safe()
    time.sleep(5)


    while (not cyber.is_shutdown() and player.Safe_PD ==1):
        now = cyber_time.Time.now().to_sec()
        player.publish_planningmsg()
        sleep_time = 0.1 - (cyber_time.Time.now().to_sec() - now)
        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()
