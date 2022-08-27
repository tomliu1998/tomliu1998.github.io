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
import queue
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
SEARCH_THETA = 100
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

        self.i = 0


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
        shortest_dist_unsqrt = 20.0
        #self.car_x = self.localization.pose.position.x
        #self.car_y = self.localization.pose.position.y
        for i_mode in range(k_cp, (k_cp+self.i_js_4-1)):
            closet_dist_unsqrt = (self.carx - self.data['x'][i_mode])**2 + (self.cary - self.data['y'][i_mode])**2
            S_close_xy = (self.data['x'][i_mode]-self.carx)*(self.data['y'][i_mode+10]-self.cary)-(self.data['y'][i_mode]-self.cary)*(self.data['x'][i_mode+10]-self.carx)

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
        if shortest_dist_unsqrt == 20.0:
            self.logger.error("can not find closet_dist")
        
        return i_cp, closet_dist

    def closet_point_LC(self, k_cp_LC):
        shortest_dist_unsqrt = 20.0
        #self.car_x = self.localization.pose.position.x
        #self.car_y = self.localization.pose.position.y
        for i_mode_LC in range(k_cp_LC, (k_cp_LC+self.i_js_2-1)):
            closet_dist_unsqrt = (self.carx - self.data['x'][i_mode_LC])**2 + (self.cary - self.data['y'][i_mode_LC])**2
            #S_close_xy = (self.data['x'][i_mode_LC]-self.carx)*(self.data['y'][i_mode_LC+10]-self.cary)-(self.data['y'][i_mode_LC]-self.cary)*(self.data['x'][i_mode_LC+10]-self.carx)

            # if S_close_xy > 0:
            #     closet_one = 1
            # elif S_close_xy < 0:
            #     closet_one = -1
            # else:
            #     closet_one = 0
            
            if closet_dist_unsqrt <= shortest_dist_unsqrt:
                shortest_dist_unsqrt = closet_dist_unsqrt
                i_cp_LC = i_mode_LC
                #closet_dist = math.sqrt(shortest_dist_unsqrt)*closet_one
        if shortest_dist_unsqrt == 20.0:
            self.logger.error("can not find closet_dist")
        
        return i_cp_LC
            





    def cal_cur(self, i_cur):
        fit_xy =  np.polyfit(self.data['x'][i_cur:i_cur+SEARCH_CUR], self.data['y'][i_cur:i_cur+SEARCH_CUR], 2)
        self.logger.info("polyfit : %s" % fit_xy)
        fit_xy_a = (1+(2*fit_xy[0]*self.data['x'][i_cur]+fit_xy[1])**2)**1.5
        self.logger.info("fit_xy_a : %s" % fit_xy_a)
        fit_xy_b = np.absolute(2*fit_xy[0])
        self.logger.info("fit_xy_b : %s" % fit_xy_b)
        # if fit_xy_a == 0.0:
        #     fit_xy_a = 0.0001
        fit_xy_cur = fit_xy_b/fit_xy_a
        return fit_xy_cur


    def cal_cur_pro(self, i_cur):
        fit_xy =  np.polyfit(self.data['x'][i_cur:i_cur+SEARCH_CUR], self.data['y'][i_cur:i_cur+SEARCH_CUR], 5)
        self.logger.info("polyfit_5 : %s" % fit_xy)
        y_diff_1 = 5*fit_xy[0]*self.data['x'][i_cur]**4 + 4*fit_xy[1]*self.data['x'][i_cur]**3 + 3*fit_xy[2]*self.data['x'][i_cur]**2 + 2*fit_xy[3]*self.data['x'][i_cur] + fit_xy[4]
        y_diff_2 = 20*fit_xy[0]*self.data['x'][i_cur]**3 + 12*fit_xy[1]*self.data['x'][i_cur]**2 + 6*fit_xy[2]*self.data['x'][i_cur] + 2*fit_xy[3]
        fit_xy_a = (1 + (y_diff_1)**2)**1.5
        fit_xy_b = np.absolute(y_diff_2)
        self.logger.info("fit_xy_b : %s" % fit_xy_b)
        # if fit_xy_a == 0.0:
        #     fit_xy_a = 0.0001
        fit_xy_cur = fit_xy_b/fit_xy_a
        return fit_xy_cur


    
    def cur_planning(self, i_cp_now):

        # num_cur = 0
        # for i_cur_plan in range(i_cp_now, i_cp_now + 9):
        #     num_cur = num_cur + self.cal_cur(i_cur_plan)
        # cur_planned_aver = num_cur/10

        cur_planned_aver = self.cal_cur(i_cp_now + 100)

        if self.data['WheelAngle_f'][i_cp_now + 100] > 0:
            cur_one = 1
        elif self.data['WheelAngle_f'][i_cp_now + 100] < 0:
            cur_one = -1
        else:
            cur_one = 0
        
        cur_planned = cur_planned_aver * cur_one

        return cur_planned


    def speed_planning(self, i_speed):
        t_planning = self.data['time'][i_speed+SPEED_PLAN_STEP] - self.data['time'][i_speed]
        S_planning_unsqrt = (self.data['x'][i_speed+SPEED_PLAN_STEP] - self.data['x'][i_speed])**2 + (self.data['y'][i_speed+SPEED_PLAN_STEP] - self.data['y'][i_speed])**2
        S_planning = math.sqrt(S_planning_unsqrt)
        speed_planning = (S_planning/t_planning)*3.6
        return speed_planning



    def speed_planning_pro(self, i_speed):

        if i_speed+self.Speed_planning_step >= self.i_first + self.i_js_4-1:
            self.Speed_planning_step = self.i_first + self.i_js_4-1 - i_speed

        t_planning = self.data['time'][i_speed+self.Speed_planning_step] - self.data['time'][i_speed]
        S_planning_unsqrt = (self.data['x'][i_speed+self.Speed_planning_step] - self.data['x'][i_speed])**2 + (self.data['y'][i_speed+self.Speed_planning_step] - self.data['y'][i_speed])**2
        S_planning = math.sqrt(S_planning_unsqrt)
        speed_planning = (S_planning/t_planning)*3.6

        if speed_planning < (self.data['speed'][i_speed] - 1):
            speed_planning = self.data['speed'][i_speed]
        return speed_planning

    


    def cal_theta(self, i_theta):
        #self.cartheta = self.localization.pose.heading
        cal_m = (self.cartheta - self.data['theta'][i_theta])
        if  cal_m > math.pi:
            car_theta = (cal_m - 2*math.pi) * 180/math.pi
        elif cal_m < -math.pi:
            car_theta = (cal_m + 2*math.pi) * 180/math.pi
        else:
            car_theta = cal_m * 180/math.pi
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
        self.logger.info("self.end = %s" % self.end)

        self.FourWheel_AD_safe = 2
        self.Tr_AD_safe = 2
        self.N_parking_safe = 2
        self.LC_AD_safe = 2
        self.Speed_planning_step = 200

        #fit_xy =  np.polyfit(self.data['y'], self.data['x'], 2)

        
        while(self.start <= self.i <= self.end):
            self.logger.info("enter: publish_planningmsg: for i in rang ")
            self.i_js_2 = 0
            self.i_js_3 = 0
            self.i_js_4 = 0
            self.i_js_5 = 0

################################################################    四轮转向循迹(mode_4 send_2)     ###############################################################################################
            if self.data['SteerMode'][self.i] == 1 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode != 4 and self.data['gear'][self.i] == 3:
                self.logger.info("enter: others to four_wheel_mode the first judge")
                # self.planningdata.brk_pedl_posn_req = 40
                # self.planning_pub.write(self.planningdata)
                # time.sleep(5)

                while(self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode != 4):
                    self.logger.info("enter: others to four_wheel_mode while")
                    self.planningdata.tgt_steer_mode_req = 2
                    self.planningdata.shift_pos_req = 2
                    self.planningdata.brk_pedl_posn_req = 0
                    self.planning_pub.write(self.planningdata)
                #time.sleep(5)

                # self.u_4 = i

                # while(self.data['SteerMode'][self.u_4] == 4): 
                #     self.i_js_4 += 1
                #     self.u_4 += 1

                if self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 4 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_strmodetransstate == 1:
                    self.FourWheel_AD_safe = 1
                    while(self.chassis_detail.schaeffler.ccu_status_1_208.ccu_gearposst != 3):
                        self.logger.info("enter: Four_wheel_mode switch gear 3  ")
                        self.planningdata.shift_pos_req = 3
                        self.planningdata.brk_pedl_posn_req = 80
                        self.planning_pub.write(self.planningdata)
                else:
                    self.FourWheel_AD_safe = 0
                    self.logger.error("FourWheel mode is not finished or steermode is not 4")
                self.logger.warning("FourWheel_AD_safe: %s", self.FourWheel_AD_safe)
        


            if (self.FourWheel_AD_safe == 1 and self.data['gear'][self.i] == 3)or \
                (self.data['SteerMode'][self.i] == 1 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 4 and self.data['gear'][self.i] == 3):
            #if self.data['SteerMode'][i] == 1 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 4 and self.data['gear'][i] == 3:
                
                self.logger.info("enter: the second judge")
                self.logger.info("i = %s" % self.i)

                self.planningdata.brk_pedl_posn_req = 0
                self.planning_pub.write(self.planningdata)
                self.logger.info("enter: four_wheel_mode brake released")
                
                self.u_4 = self.i
                self.i_first = float('inf')
                
                while(self.data['SteerMode'][self.u_4] == 1 and self.data['gear'][self.u_4] == 3):
                    self.logger.info("enter: the second judge: counter ")
                    
                    self.i_js_4 += 1

                    if self.u_4 <= self.i_first:
                        self.i_first = self.u_4

                    self.u_4 += 1


                    # if self.u_4 == self.end-3:
                    #     break
                        
                if self.i_first == float('inf'):
                    self.logger.error("can not find i_first")
                
                self.logger.info("i_first: %s", self.i_first)
                self.logger.info("i_js_4: %s", self.i_js_4)


            while((self.FourWheel_AD_safe == 1 and self.data['gear'][self.i] == 3) or \
                (self.data['SteerMode'][self.i] == 1 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 4 and self.data['gear'][self.i] == 3)):
            #while(self.data['SteerMode'][i] == 1 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 4 and self.data['gear'][i] == 3):

                self.logger.info("enter: fourwheelmodel: main_while ")

                self.planningdata.tgt_steer_mode_req = 2
                i_now, self.planningdata.lateral_dis_deviation  = self.closet_point(self.i_first)
                self.planningdata.tgt_route_curvature = self.cur_planning(i_now)
                self.planningdata.headingangle_deviation = self.cal_theta(i_now + SEARCH_THETA)
                self.planningdata.tgt_long_speed = self.speed_planning(i_now)
                self.planningdata.shift_pos_req = 3

                self.logger.info("i_now: %s" % i_now)
                self.logger.info("lateral dis_deviation: %s" % self.planningdata.lateral_dis_deviation)
                self.logger.info("route curvate: %s" % self.planningdata.tgt_route_curvature)
                self.logger.info("planning speed: %s" % self.planningdata.tgt_long_speed)
                self.logger.info("act steer mode: %s" % self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode)
                self.logger.info("headingangle: %s" % self.planningdata.headingangle_deviation)
                self.logger.info("data_SteerMode: %s" % self.data['SteerMode'][self.i])
                self.logger.info("data_gear: %s" % self.data['gear'][self.i])
                
                self.planning_pub.write(self.planningdata)
                
                self.i = i_now
               
                judge_dist = math.sqrt((self.carx - self.data['x'][self.i_first+self.i_js_4-1])**2 + (self.cary - self.data['y'][self.i_first+self.i_js_4-1])**2)
                self.logger.info("judge_dist: %s" % judge_dist)

                if math.sqrt((self.carx - self.data['x'][self.i_first+self.i_js_4-1])**2 + (self.cary - self.data['y'][self.i_first+self.i_js_4-1])**2) <= 0.3:

                    judge_dist = math.sqrt((self.carx - self.data['x'][self.i_first+self.i_js_4-1])**2 + (self.cary - self.data['y'][self.i_first+self.i_js_4-1])**2)
                    self.logger.info("judge_dist_final: %s" % judge_dist)


                    while(self.chassis.wheel_speed.vehicle_spd >= 0.0):
                        self.planningdata.brk_pedl_posn_req = 80
                        self.planningdata.shift_pos_req = 2
                        self.planning_pub.write(self.planningdata)
                        self.logger.info("Four_wheel_mode still breaking")


                    self.planningdata.brk_pedl_posn_req = 0
                    self.planning_pub.write(self.planningdata)
                    self.FourWheel_AD_safe = 2
                    self.i = self.i_first + (self.i_js_4 - 1)
                    self.logger.info("i_first + i_js_4-1: %s" % self.i)
                    self.logger.info("Four_wheel_mode breaking finished")
                    break


####################################################   原地转向模式切换(mode_5 send_5)    ################################################################################################
            if self.data['SteerMode'][self.i] == 5 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode != 5 and self.data['gear'][self.i] == 3\
                or self.data['SteerMode'][self.i] == 5 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode != 5 and self.data['gear'][self.i] == 1:

                self.logger.info("enter: the Tr_mode first judge")
                self.logger.info("mode_5_gear_3 i: %s" % self.i)
                
                while(self.chassis.wheel_speed.vehicle_spd >= 0.02):
                    self.logger.info("enter: the Tr_mode first braking")
                    self.planningdata.brk_pedl_posn_req = 80
                    self.planning_pub.write(self.planningdata)

                while(self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode != 5):
                    self.logger.info("enter: Tr_mode switch while ")
                    self.planningdata.tgt_steer_mode_req = 5
                    self.planningdata.tgt_long_speed = 0
                    self.planningdata.tgt_steer_angle_req = 0
                    self.planningdata.shift_pos_req = 2
                    self.planningdata.brk_pedl_posn_req = 0
                    self.planning_pub.write(self.planningdata)



                if self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 5 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_strmodetransstate == 1:
                #if self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 5:
                    self.logger.info("enter: Tr_AD_safe = 1 ")
                    self.Tr_AD_safe = 1
                    while(self.chassis_detail.schaeffler.ccu_status_1_208.ccu_gearposst != int(self.data['gear'][self.i])):
                        self.logger.info("enter: Tr_mode switch gear 3 or 1 ")
                        self.logger.info("Tr_mode chassis gear pose: %s" % self.chassis_detail.schaeffler.ccu_status_1_208.ccu_gearposst)
                        self.logger.info("Tr_mode data gear pose: %s" % self.data['gear'][self.i])
                        
                        self.planningdata.shift_pos_req = int(self.data['gear'][self.i])
                        self.planningdata.brk_pedl_posn_req = 80
                        self.planning_pub.write(self.planningdata)
                else:
                    self.logger.info("enter: Tr_AD_safe = 0 ")
                    self.Tr_AD_safe = 0
                    self.logger.error("turning circle is not finished or steermode is not 5")
                self.logger.warning("Tr_AD_safe: %s" % self.Tr_AD_safe)
                #time.sleep(5)

            if (self.Tr_AD_safe == 1 and self.data['gear'][self.i] == 3) or (self.Tr_AD_safe == 1 and self.data['gear'][self.i] == 1) or\
                 (self.data['SteerMode'][self.i] == 5 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 5 and self.data['gear'][self.i] == 3) or \
                    (self.data['SteerMode'][self.i] == 5 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 5 and self.data['gear'][self.i] == 1):
                
                self.u_5 = self.i
                self.i_first_Tr_mode = float('inf')

                self.planningdata.brk_pedl_posn_req = 0
                self.planning_pub.write(self.planningdata)
                self.logger.info("enter: Tr_mode brake released")

                while((self.data['SteerMode'][self.u_5] == 5 and self.data['gear'][self.u_5] == 3) or \
                    (self.data['SteerMode'][self.u_5] == 5 and self.data['gear'][self.u_5] == 1)):
                    
                    self.logger.info("enter: Tr_mode counter")
                    self.i_js_5 += 1

                    if self.u_5 <= self.i_first_Tr_mode:
                        self.i_first_Tr_mode = self.u_5

                    self.u_5 += 1

                if self.i_first_Tr_mode == float('inf'):
                    self.logger.error("can not find i_first_Tr_mode")

                self.logger.info("i_js_5: %s", self.i_js_5)
                self.logger.info("i_first_Tr_mode: %s", self.i_first_Tr_mode)

            

            
            while((self.Tr_AD_safe == 1 and self.data['gear'][self.i] == 3) or (self.Tr_AD_safe == 1 and self.data['gear'][self.i] == 1) or\
                 (self.data['SteerMode'][self.i] == 5 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 5 and self.data['gear'][self.i] == 3) or \
                    (self.data['SteerMode'][self.i] == 5 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 5 and self.data['gear'][self.i] == 1)):

                self.logger.info("enter: Tr_mode main while")

                if self.data['gear'][self.i] == 3:
                    Tr_mode_speed_one = 1
                elif self.data['gear'][self.i] == 1:
                    Tr_mode_speed_one = -1

                # self.u_5 = self.i

                # while((self.data['SteerMode'][self.u_5] == 5 and self.data['gear'][self.u_5] == 3) or \
                #     (self.data['SteerMode'][self.u_5] == 5 and self.data['gear'][self.u_5] == 1)):
                #     self.logger.info("enter: Tr_mode counter while")
                #     self.i_js_5 += 1
                #     self.u_5 += 1

                #self.logger.info("i_js_5: %s", self.i_js_5)
                self.planningdata.tgt_steer_mode_req = 5
                self.planningdata.tgt_long_speed = 1*Tr_mode_speed_one
                self.planningdata.tgt_steer_angle_req = 0
                self.planningdata.shift_pos_req = int(self.data['gear'][self.i])
                #self.planningdata.shift_pos_req =  3
                self.planning_pub.write(self.planningdata)

                #while(self.localization.pose.heading == self.data['theta'][(i+self.i_js_5-1)]):
                while(np.absolute(self.cal_theta(self.i+self.i_js_5-1)) <= 5.0):

                    self.cal_theta_Tr_mode_braking  =  np.absolute(self.cal_theta(self.i+self.i_js_5-1))
                    self.logger.info("cal_theta_Tr_mode_braking: %s", self.cal_theta_Tr_mode_braking)
                    self.logger.info("enter: Tr_mode angle judge")

                    self.planningdata.brk_pedl_posn_req = 80
                    self.planning_pub.write(self.planningdata)
                    if self.chassis.wheel_speed.vehicle_spd == 0:
                        self.i = self.i+self.i_js_5
                        self.logger.info("i + i_js_5: %s", self.i)
                        self.Tr_AD_safe = 2
                        self.planningdata.brk_pedl_posn_req = 0
                        self.planning_pub.write(self.planningdata)
                        self.logger.info("Tr_Mode breaking finished")
                        break
                    
            
            

            # while(self.data['SteerMode'][i] == 5 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 5):
            #     self.logger.info("enter: Tr_mode remain judge")
            #     i += 1
            #     if self.data['SteerMode'][i] != 5:
            #         self.logger.info("remain_judge_i: %s", i)
            #         break
        
        
####################################################   90度平移模式切换(mode_3 send_4)    ################################################################################################
            if self.data['SteerMode'][self.i] == 3 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode != 3 and self.data['gear'][self.i] == 3\
                or self.data['SteerMode'][self.i] == 3 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode != 3 and self.data['gear'][self.i] == 1:

                self.logger.info("enter: the N_parking_mode first judge")
                self.logger.info("mode_3_gear_3 i: %s" % self.i)
                
                while(self.chassis.wheel_speed.vehicle_spd >= 0.02):
                    self.logger.info("enter: the N_parking_mode first braking")
                    self.planningdata.brk_pedl_posn_req = 80
                    self.planning_pub.write(self.planningdata)

                while(self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode != 3):
                    self.logger.info("enter: N_parking_mode switch while ")
                    self.planningdata.tgt_steer_mode_req = 4
                    self.planningdata.tgt_long_speed = 0
                    self.planningdata.tgt_steer_angle_req = 0
                    self.planningdata.shift_pos_req = 2
                    self.planningdata.brk_pedl_posn_req = 0
                    self.planning_pub.write(self.planningdata)



                if self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 3 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_strmodetransstate == 1:
                    self.logger.info("enter: N_parking_safe = 1 ")
                    self.N_parking_safe = 1
                    while(self.chassis_detail.schaeffler.ccu_status_1_208.ccu_gearposst != int(self.data['gear'][self.i])):
                        self.logger.info("enter: N_parking_mode switch gear 3 or 1 ")
                        self.logger.info("N_parking_mode chassis gear pose: %s" % self.chassis_detail.schaeffler.ccu_status_1_208.ccu_gearposst)
                        self.logger.info("N_parking_mode data gear pose: %s" % self.data['gear'][self.i])

                        self.planningdata.shift_pos_req = int(self.data['gear'][self.i])
                        self.planningdata.brk_pedl_posn_req = 80
                        self.planning_pub.write(self.planningdata)
                else:
                    self.logger.info("enter: N_parking_safe = 0 ")
                    self.N_parking_safe = 0
                    self.logger.error("turning circle is not finished or steermode is not 3")
                self.logger.warning("N_parking_safe: %s" % self.N_parking_safe)



            if (self.N_parking_safe == 1 and self.data['gear'][self.i] == 3) or (self.N_parking_safe == 1 and self.data['gear'][self.i] == 1) or\
                 (self.data['SteerMode'][self.i] == 3 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 3 and self.data['gear'][self.i] == 3) or \
                    (self.data['SteerMode'][self.i] == 3 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 3 and self.data['gear'][self.i] == 1):
                
                
                self.u_3 = self.i
                self.i_first_N_parking = float('inf')

                self.planningdata.brk_pedl_posn_req = 0
                self.planning_pub.write(self.planningdata)
                self.logger.info("enter: N_parking_mode brake released")

                while((self.data['SteerMode'][self.u_3] == 3 and self.data['gear'][self.u_3] == 3) or \
                    (self.data['SteerMode'][self.u_3] == 3 and self.data['gear'][self.u_3] == 1)):

                    self.logger.info("enter: N_parking_mode counter")
                    self.i_js_3 += 1

                    if self.u_3 <= self.i_first_N_parking:
                        self.i_first_N_parking = self.u_3

                    self.u_3 += 1

                if self.i_first_N_parking == float('inf'):
                    self.logger.error("can not find i_first_N_parking")
                
                self.logger.info("i_first_N_parking: %s", self.i_first_N_parking)
                self.logger.info("i_js_3: %s", self.i_js_3)


                

            while((self.N_parking_safe == 1 and self.data['gear'][self.i] == 3) or (self.N_parking_safe == 1 and self.data['gear'][self.i] == 1) or\
                 (self.data['SteerMode'][self.i] == 3 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 3 and self.data['gear'][self.i] == 3) or \
                    (self.data['SteerMode'][self.i] == 3 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 3 and self.data['gear'][self.i] == 1)):
                
                self.logger.info("enter: N_parking_mode main while")

                if self.data['gear'][self.i] == 3:
                    N_parking_mode_speed_one = 1
                elif self.data['gear'][self.i] == 1:
                    N_parking_mode_speed_one = -1

                # self.u_3 = self.i
                # self.i_first_N_parking = float('inf')

                # while((self.data['SteerMode'][self.u_3] == 3 and self.data['gear'][self.u_3] == 3) or \
                #     (self.data['SteerMode'][self.u_3] == 3 and self.data['gear'][self.u_3] == 1)):

                #     self.logger.info("enter: N_parking_mode counter while")
                #     self.i_js_3 += 1

                #     if self.u_3 <= self.i_first_N_parking:
                #         self.i_first_N_parking = self.u_3

                #     self.u_3 += 1

                # if self.i_first_N_parking == float('inf'):
                #     self.logger.error("can not find i_first_N_parking")
                
                # self.logger.info("i_first_N_parking: %s", self.i_first_N_parking)
                # self.logger.info("i_js_3: %s", self.i_js_3)

                self.planningdata.tgt_steer_mode_req = 4
                self.planningdata.tgt_long_speed = 1*N_parking_mode_speed_one
                self.planningdata.tgt_steer_angle_req = 0
                self.planningdata.shift_pos_req = int(self.data['gear'][self.i])
                self.planning_pub.write(self.planningdata)

                if math.sqrt((self.carx - self.data['x'][self.i_first_N_parking+self.i_js_3-1])**2 + (self.cary - self.data['y'][self.i_first_N_parking+self.i_js_3-1])**2) <= 0.5:

                    judge_dist_N_parking = math.sqrt((self.carx - self.data['x'][self.i_first_N_parking+self.i_js_3-1])**2 + (self.cary - self.data['y'][self.i_first_N_parking+self.i_js_3-1])**2)
                    self.logger.info("N_parking_judge_dist_final: %s" % judge_dist_N_parking)


                    #while(np.absolute(self.chassis.wheel_speed.vehicle_spd) >= 0.0):
                    while(self.chassis.wheel_speed.vehicle_spd != 0):
                        self.logger.info("N_parking_mode braking_speed: %s" % self.chassis.wheel_speed.vehicle_spd)
                        self.planningdata.brk_pedl_posn_req = 40
                        self.planningdata.shift_pos_req = 2
                        self.planning_pub.write(self.planningdata)

                    self.planningdata.brk_pedl_posn_req = 0
                    self.planning_pub.write(self.planningdata)
                    self.N_parking_safe = 2
                    self.i = self.i_first_N_parking + (self.i_js_3-1)
                    self.logger.info("i_first_N_parking + i_js_3 - 1: %s" % self.i)
                    self.logger.info("N_parking_mode breaking finished")
                    break
            




            ####################################################   蟹行模式切换(mode_2 send_3)    ################################################################################################
            if self.data['SteerMode'][self.i] == 2 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode != 2 and self.data['gear'][self.i] == 3:

                self.logger.info("enter: others to LC_mode the first judge")
                self.logger.info("mode_2_gear_3_i: %s" % self.i)


                while(self.chassis.wheel_speed.vehicle_spd >= 0.02):
                    self.logger.info("enter: the LC_mode first braking")
                    self.planningdata.brk_pedl_posn_req = 80
                    self.planning_pub.write(self.planningdata)

                while(self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode != 2):
                    self.logger.info("enter: others to LC_mode while")
                    self.planningdata.tgt_steer_mode_req = 3
                    self.planningdata.tgt_long_speed = 0
                    self.planningdata.tgt_steer_angle_req = 0
                    self.planningdata.shift_pos_req = 2
                    self.planningdata.brk_pedl_posn_req = 0
                    self.planning_pub.write(self.planningdata)
                

                if self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 2 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_strmodetransstate == 1:
                    self.LC_AD_safe = 1
                    while(self.chassis_detail.schaeffler.ccu_status_1_208.ccu_gearposst != 3):
                        self.logger.info("enter: LC_mode switch gear 3 ")
                        self.logger.info("LC_mode chassis gear pose: %s" % self.chassis_detail.schaeffler.ccu_status_1_208.ccu_gearposst)

                        self.planningdata.shift_pos_req = 3
                        self.planningdata.brk_pedl_posn_req = 80
                        self.planning_pub.write(self.planningdata)
                else:
                    self.LC_AD_safe = 0
                    self.logger.error("LC mode is not finished or steermode is not 4")
                self.logger.warning("LC_AD_safe: %s", self.LC_AD_safe)


            if (self.LC_AD_safe == 1 and self.data['gear'][self.i] == 3) or \
                (self.data['SteerMode'][self.i] == 2 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 2 and self.data['gear'][self.i] == 3):
                
                self.u_2 = self.i
                self.i_first_LC = float('inf')

                self.planningdata.brk_pedl_posn_req = 0
                self.planning_pub.write(self.planningdata)
                self.logger.info("enter: LC_mode brake released")

                while(self.data['SteerMode'][self.u_2] == 2 and self.data['gear'][self.u_2] == 3):

                    self.logger.info("enter: LC_mode counter")
                    self.i_js_2 += 1

                    if self.u_2 <= self.i_first_LC:
                        self.i_first_LC = self.u_2

                    self.u_2 += 1

                if self.i_first_LC == float('inf'):
                    self.logger.error("can not find i_first_LC")
                
                self.logger.info("i_first_LC: %s", self.i_first_LC)
                self.logger.info("i_js_2: %s", self.i_js_2)

            while((self.LC_AD_safe == 1 and self.data['gear'][self.i] == 3) or (self.data['SteerMode'][self.i] == 2 and self.chassis_detail.schaeffler.ccu_status_1_208.ccu_actsteermode == 2 and self.data['gear'][self.i] == 3)):
                
                self.logger.info("enter: LC_mode main while")

                self.planningdata.tgt_steer_mode_req = 3
                i_now_LC = self.closet_point_LC(self.i_first_LC)
                self.planningdata.tgt_long_speed = 2
                self.planningdata.shift_pos_req = 3
                self.planningdata.tgt_steer_angle_req = self.data['WheelAngle_f'][i_now_LC + 30]
                

                self.logger.info("i_now_LC: %s" % i_now_LC)
                self.logger.info("i_now_LC: %s" % self.planningdata.tgt_steer_angle_req)
                self.logger.info("tgt_steer_angle: %s" % self.planningdata.tgt_steer_angle_req)
                self.logger.info("data_wheelangle_f: %s" % self.data['WheelAngle_f'][i_now_LC])
                
                self.planning_pub.write(self.planningdata)

                self.i = i_now_LC

                judge_dist_LC = math.sqrt((self.carx - self.data['x'][self.i_first_LC+self.i_js_2-1])**2 + (self.cary - self.data['y'][self.i_first_LC+self.i_js_2-1])**2)
                self.logger.info("judge_dist_LC: %s" % judge_dist_LC)

                if judge_dist_LC <= 0.3:
                    self.logger.info("judge_dist_final: %s" % judge_dist_LC)

                    while(self.chassis.wheel_speed.vehicle_spd >= 0.0):
                        self.planningdata.brk_pedl_posn_req = 80
                        self.planningdata.shift_pos_req = 2
                        self.planning_pub.write(self.planningdata)
                        self.logger.info("LC_mode still breaking")
                    
                    self.planningdata.brk_pedl_posn_req = 0
                    self.planning_pub.write(self.planningdata)
                    self.LC_AD_safe = 2
                    self.i = self.i_first_LC + (self.i_js_2-1)
                    self.logger.info("i_first_LC + i_js_2 - 1: %s" % self.i)
                    self.logger.info("LC_mode breaking finished")
                    break


                









            self.i += 1

                































            








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
