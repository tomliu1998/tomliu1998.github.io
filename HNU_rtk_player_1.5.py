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

# TODO(all): hard-coded path temporarily. Better approach needed.
APOLLO_ROOT = "/apollo"

SEARCH_INTERVAL = 1000
SEARCH_CUR = 500
CHANGE_TO_COM = False


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
        self.padmsg = pad_msg_pb2.PadMessage()
        self.localization_received = False
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
        
    def padmsg_callback(self, data):
        """
        New message received
        """
        if self.terminating is True:
            self.logger.info("terminating when receive padmsg")
            return

        self.padmsg.CopyFrom(data)


    def lateral_dist(self, d_x, d_y):
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
        if shortest_dist_sqr == float('inf'):
            self.logger.info(
                'can not find closet point')
        #有关水平偏差的正负值以及单位(m)还需确认
        return lateral_dist_sqrt
    
    def cal_cur(self, c_y):
        fit_xy =  np.polyfit(self.data['y'], self.data['x'], 2)
        for i in range(self.start, self.end):
            fit_cur_r= (1+(2*fit_xy[0]*c_y[i]+fit_xy[1])**2)**1.5/2*fit_xy[0]

        return fit_cur_r
            
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





    def publish_planningmsg(self):
        """
        Generate New Path
        """
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
            #adc_point = planningdata
            #adc_point.path_point.x = self.data['x'][i]
            #adc_point.path_point.y = self.data['y'][i]
            #adc_point.path_point.z = self.data['z'][i]
            self.planningdata.path_point.heading = self.data['theta'][i]
            self.planningdata.lateral_dis_deviation = self.lateral_dist(self.data['x'], self.data['y'])
            self.planningdata.tgt_long_speed = self.data['speed'][i]
            self.planningdata.shift_pos_req = self.data['gear'][i]
            fit_xy =  np.polyfit(self.data['y'][i:i+SEARCH_CUR], self.data['x'][i:i+SEARCH_CUR], 2)
            fit_xy_cur = (1+(2*fit_xy[0]*self.data['y'][i]+fit_xy[1])**2)**1.5/math.sqrt(2*fit_xy[0])
            if self.data['WheelAngle_f'][i] > 0:
                cur_one = 1
            elif self.data['WheelAngle_f'][i] < 0:
                cur_one = -1
            else:
                cur_one = 0
            self.planningdata.tgt_route_curvature = fit_xy_cur*cur_one

            #planningdata.trajectory_point.extend([adc_point])

            self.planning_pub.write(planningdata)
            self.logger.debug("Generated Planning Sequence: "
                          + str(self.sequence_num - 1))

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

    node.create_reader('/apollo/localization/pose',
                       localization_pb2.LocalizationEstimate,
                       player.localization_callback)

    node.create_reader('/apollo/control/pad', pad_msg_pb2.PadMessage,
                       player.padmsg_callback)

    player.Standby_PD()
    #time.sleep(2)
    player.Parking_rels()
    #time.sleep(2)
    player.AD_safe()
    #time.sleep(2)


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
