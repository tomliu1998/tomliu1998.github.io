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

import atexit
import logging
import math
import os
import sys
import time
#from turtle import speed

from cyber.python.cyber_py3 import cyber
from gflags import FLAGS

from modules.tools.common.logger import Logger
from modules.canbus.proto import chassis_pb2
from modules.localization.proto import localization_pb2


class RtkRecord(object):
    """
    rtk recording class
    """

    def write(self, data):
        """Wrap file write function to flush data to disk"""
        self.file_handler.write(data)
        self.file_handler.flush()

    def __init__(self, record_file):
        self.firstvalid = False
        self.logger = Logger.get_logger("RtkRecord")
        self.record_file = record_file
        self.logger.info("Record file to: " + record_file)

        try:
            self.file_handler = open(record_file, 'w') #'W':open for writing, truncating the file first
        except IOError:
            self.logger.error("Open file %s failed" % (record_file))
            self.file_handler.close()
            sys.exit(1)

        #向record_file中写入数据，就是第一行写上变量名
        self.write("x,y,z,theta,time,speed,gear\n")

        self.localization = localization_pb2.LocalizationEstimate()
        self.chassis = chassis_pb2.Chassis()
        self.chassis_received = False

        self.cars = 0.0
        self.startmoving = False

        self.terminating = False
        self.carcurvature = 0.0

        self.prev_carspeed = 0.0

    def chassis_callback(self, data):
        """
        New message received
        """
        if self.terminating is True:
            self.logger.info("terminating when receive chassis msg")
            return

        self.chassis.CopyFrom(data)
        #self.chassis = data
        if math.isnan(self.chassis.speed_mps):
            self.logger.warning("find nan speed_mps: %s" % str(self.chassis))
        '''if math.isnan(self.chassis.steering_percentage):
            self.logger.warning(
                "find nan steering_percentage: %s" % str(self.chassis))'''
        self.chassis_received = True

    def localization_callback(self, data):
        """
        New message received
        """
        if self.terminating is True:
            self.logger.info("terminating when receive localization msg")
            return

        if not self.chassis_received:
            self.logger.info(
                "chassis not received when localization is received")
            return

        #将定位数据传入成员变量self.localization
        self.localization.CopyFrom(data)
        #self.localization = data

        #读取本车位置的x,y,z坐标，该坐标是在UTM坐标系下的
        carx = self.localization.pose.position.x
        cary = self.localization.pose.position.y
        carz = self.localization.pose.position.z
        cartheta = self.localization.pose.heading
        if math.isnan(self.chassis.speed_mps):
            self.logger.warning("find nan speed_mps: %s" % str(self.chassis))
            return
        '''if math.isnan(self.chassis.steering_percentage):
            self.logger.warning(
                "find nan steering_percentage: %s" % str(self.chassis))
            return'''

        #与接口组确定chassis模块获取速度信息的地址
        carspeed = self.chassis.speed_mps
        #caracceleration = self.localization.pose.linear_acceleration_vrf.y

        '''speed_epsilon = 1e-9
        if abs(self.prev_carspeed) < speed_epsilon \
                and abs(carspeed) < speed_epsilon:
            caracceleration = 0.0'''

        #carsteer = self.chassis.steering_percentage
        
        #曲率
        '''curvature = math.tan(math.radians(carsteer / 100 * 470) / 16) / 2.85
        if abs(carspeed) >= speed_epsilon:
            carcurvature_change_rate = (curvature - self.carcurvature) / (
                carspeed * 0.01)
        else:
            carcurvature_change_rate = 0.0
        self.carcurvature = curvature'''
        cartime = self.localization.header.timestamp_sec
        #与接口组确定chassis模块获取速度信息的地址
        cargear = self.chassis.gear_location

        '''if abs(carspeed) >= speed_epsilon:
            if self.startmoving is False:
                self.logger.info(
                    "carspeed !=0 and startmoving is False, Start Recording")
            self.startmoving = True'''

        #向文件写入数据
        self.write(
            "%s, %s, %s, %s, %.4f, %s, %s\n" %
            (carx, cary, carz, cartheta, cartime, carspeed, cargear))
        self.logger.debug(
            "started moving and write data at time %s" % cartime)


    def shutdown(self):
        """
        shutdown node
        """
        self.terminating = True
        self.logger.info("Shutting Down...")
        self.logger.info("File is written into %s" % self.record_file)
        self.file_handler.close()


def main(argv):
    """
    Main node
    """
    node = cyber.Node("rtk_recorder")
    argv = FLAGS(argv)

    log_dir = "/apollo/data/log" #建立文件夹
    if len(argv) > 1:
        log_dir = argv[1]

    if not os.path.exists(log_dir):
        os.makedirs(log_dir)

    Logger.config(
        log_file=log_dir + "rtk_recorder.log",  #文件记录数据
        use_stdout=True,
        log_level=logging.DEBUG)
    print("runtime log is in %s%s" % (log_dir, "rtk_recorder.log"))

    record_file = log_dir + "/garage.csv"
    recorder = RtkRecord(record_file)
    atexit.register(recorder.shutdown)

    node.create_reader('/apollo/canbus/chassis',
                       chassis_pb2.Chassis,
                       recorder.chassis_callback)

    node.create_reader('/apollo/localization/pose',
                       localization_pb2.LocalizationEstimate,
                       recorder.localization_callback)

    while not cyber.is_shutdown():
        time.sleep(0.002)


if __name__ == '__main__':
    cyber.init()
    main(sys.argv)
    cyber.shutdown()

