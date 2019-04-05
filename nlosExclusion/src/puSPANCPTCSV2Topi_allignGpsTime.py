#!/usr/bin/env python
# license removed for brevity
"""
    CSV to Topic
    Welson Wen, Ph.D.
    https://sites.google.com/view/weisongwen/about-me

    subcribe : '/novatel_data/bestpos' get gps_week_second
                '/imu_c' get time stamp
    publish: 'GNSS_' GNSS data provided by ublox M8T with rostopic time stamp alligned with novatel ros msg
    parameters needed: begin GPS_week_time and end GPS_week_time in bag file
"""
import csv # csv reading needed library
import datetime #time format (datetime)
import time #time format (time)
from sensor_msgs.msg   import NavSatFix # standard message type for GNSSs
from nlosExclusion.msg import GNSS_Raw_Array,GNSS_Raw # customerized ros message type
from matplotlib.patches import Ellipse, Circle # draw circle needs library
import rospy #ros python needed
import ecef2llh #ecef coordinate to llh coordinate
from std_msgs.msg import String #std message
from numpy import * # numpy needed
import matplotlib as mpl #plot needed
from novatel_msgs.msg import BESTPOS
mpl.use("TkAgg") # Use TKAgg to show figures:set this to show plot
import matplotlib.pyplot as plt #plotting
import pandas as pd # pandas needed renamed as pd
import numpy as np #numpy needed renamed as np
import geometry_msgs.msg as gm #ros geometry message
from geometry_msgs.msg import Quaternion, Point, Pose, Twist,PoseArray # commonly used message type
from PyQt4 import QtCore, QtGui
import puGNSSPosCal
from sensor_msgs.msg   import Imu # standard message type for GNSSs

#read csv file into rostopic (GNSS Raw data)
class csv2Topi():
    def __init__(self):
        # rospy.init_node('csv2Topi_', anonymous=True)  # initialize node with specific node name
        self.GNSS_Raw_pub = rospy.Publisher('GNSS_', GNSS_Raw_Array,
                                       queue_size=100)  # customerized GNSS raw data ros message
        rospy.Subscriber('/novatel_data/bestpos', BESTPOS, self.callcptBestPos_)
        rospy.Subscriber('/imu_c', Imu, self.callcptImu)
        self.lowEleSatLis_ = [3, 91, 22, 32, 94]
        self.lowEleSatLis_ = []
        self.eleThres_ = 18.0 # 18
        self.snrThres_ = 20.0 # 20
        self.GNSSArr_ = GNSS_Raw_Array()
        self.initVariable()

    def callcptImu(self,data):
        self.Imu_ = Imu()
        self.Imu_ = data
        self.seq = self.Imu_.header.seq
        self.stampSecs = self.Imu_.header.stamp.secs
        self.stampnSecs = self.Imu_.header.stamp.nsecs

        self.Imu_.linear_acceleration.z = self.Imu_.linear_acceleration.z + 9.7803
        # self.ImuPub_.publish(self.Imu_)

    def callcptBestPos_(self,data):
        self.bestPos_ = BESTPOS()
        self.bestPos_ = data
        self.GNSSArr1 = GNSS_Raw_Array()
        pos_index = 0
        if(self.endFlag == 1):
            if((self.bestPos_.header.gps_week_seconds) in self.GNSS_time):
                print 'GNSS_time.index(self.bestPos_.header.gps_week_seconds)',self.bestPos_.header.gps_week_seconds
                print self.GNSS_time.index(self.bestPos_.header.gps_week_seconds)
                print '-------------------------------------------'
                pos_index = self.GNSS_time.index(self.bestPos_.header.gps_week_seconds)
                totalSv_ = self.total_sv[pos_index]
                self.GNSS_1 = GNSS_Raw()
                for index_ in range(int(totalSv_)):
                    index_ = index_ + pos_index
                    print 'index_',index_
                    self.GNSS_1.GNSS_time = float(self.GNSS_time[index_])  # GNSS time contained in csv file
                    self.GNSS_1.total_sv = float(self.total_sv[index_])  # total sitellites in one epoch (epoch: each time point)
                    self.GNSS_1.prn_satellites_index = float(self.prn_satellites_index[index_])
                    self.GNSS_1.pseudorange = float(self.pseudorange[index_])
                    self.GNSS_1.snr = float(self.snr[index_])  # signal noise ratio for this satellite
                    self.GNSS_1.elevation = float(self.elevation[index_])  # elevation for this satellite and receiver
                    self.GNSS_1.azimuth = float(self.azimuth[index_])  # azimuth for this satellite
                    self.GNSS_1.err_tropo = float(self.err_tropo[index_])  # troposphere error in meters
                    self.GNSS_1.err_iono = float(self.err_iono[index_])  # ionophere error in meters
                    self.GNSS_1.sat_clk_err = float(self.sat_clk_err[index_])  # satellite clock bias caused error
                    self.GNSS_1.sat_pos_x = float(self.sat_pos_x[index_])  # satellite positioning in x direction
                    self.GNSS_1.sat_pos_y = float(self.sat_pos_y[index_])  # satellite positioning in y direction
                    self.GNSS_1.sat_pos_z = float(self.sat_pos_z[index_])  # satellite positioning in Z direction
                    if(self.GNSS_1.elevation > 18.0 ): # mask angle = 18
                        self.GNSSArr1.GNSS_Raws.append(self.GNSS_1)
                    self.GNSS_1 = GNSS_Raw()

                self.GNSSArr1.header.seq = self.seq
                self.GNSSArr1.header.stamp.secs = self.stampSecs
                self.GNSSArr1.header.stamp.nsecs = self.stampnSecs
                self.GNSSArr1.header.frame_id = 'GNSS_'
                self.GNSS_Raw_pub.publish(self.GNSSArr1)


    def initVariable(self):
        self.seq = 0.0
        self.stampSecs = 0.0
        self.stampnSecs = 0.0
        self.GNSS_time = []  # GNSS time contained in csv file
        self.total_sv = []  # total sitellites in one epoch (epoch: each time point)
        self.prn_satellites_index = []  # satellite index in this epoch (epoch: each time point)
        self.pseudorange = []  # pseudorange measurement
        self.snr = []  # signal noise ratio for this satellite
        self.elevation = []  # elevation for this satellite and receiver
        self.azimuth = []  # azimuth for this satellite
        self.err_tropo = []  # troposphere error in meters
        self.err_iono = []  # ionophere error in meters
        self.sat_clk_err = []  # satellite clock bias caused error
        self.sat_pos_x = []  # satellite positioning in x direction
        self.sat_pos_y = []  # satellite positioning in y direction
        self.sat_pos_z = []  # satellite positioning in Z direction
        self.begFlag = 0
        self.endFlag = 0

    def fastReadCSV(self,GNSSBegin, GNSSEnd):

        # self.Fcsv_GNSS = csv.reader(open('/home/wenws/20180701/Niutoujiao/sv_dataSlow.csv',
        #                                 'r'))  # read csv context to csv_reader variable
        # self.Fcsv_GNSS = csv.reader(open('/home/wenws/CV_GNSS/src/ros_skymask/ivactuator/data/sv_data20181206.csv',
        #                                 'r'))  # read csv context to csv_reader variable
        self.Fcsv_GNSS = csv.reader(open('/media/wenws/TOSHIBA EXT/insidegnss/sv_data20190326.csv',
                                        'r'))  # read csv context to csv_reader variable
        for rowCsv in self.Fcsv_GNSS:
            if(rowCsv[0] == 'gps_tow(sec)'):
                continue
            if((int(rowCsv[0]) * 1000) == GNSSBegin):
                print 'detect first frame and begin first frame process'
                self.begFlag = 1
            if ((int(rowCsv[0]) * 1000) >= GNSSEnd):
                print 'detect final frame and finish end frame process'
                print 'length',len(self.GNSS_time)
                print 'you can open the rosbag file now, then subscribe /novatel_data/bestpos'
                # print 'self.prn_satellites_index',self.prn_satellites_index
                self.endFlag = 1
                break
                # print rowCsv
            if(self.begFlag == 1):
                self.GNSS_time.append(float(rowCsv[0]) * 1000)  # GNSS time contained in csv file
                self.total_sv.append(rowCsv[1])  # total sitellites in one epoch (epoch: each time point)
                self.prn_satellites_index.append(
                    rowCsv[2])  # satellite index in this epoch (epoch: each time point)
                self.pseudorange.append(float(rowCsv[3]) - float(rowCsv[7]) - float(rowCsv[8]) + float(
                    rowCsv[9]))  # pseudorange measurement
                self.snr.append(rowCsv[4])  # signal noise ratio for this satellite
                self.elevation.append(rowCsv[5])  # elevation for this satellite and receiver
                self.azimuth.append(rowCsv[6])  # azimuth for this satellite
                self.err_tropo.append(rowCsv[7])  # troposphere error in meters
                self.err_iono.append(rowCsv[8])  # ionophere error in meters
                self.sat_clk_err.append(rowCsv[9])  # satellite clock bias caused error
                self.sat_pos_x.append(rowCsv[10])  # satellite positioning in x direction
                self.sat_pos_y.append(rowCsv[11])  # satellite positioning in y direction
                self.sat_pos_z.append(rowCsv[12])  # satellite positioning in Z direction

if __name__ == '__main__':
    rospy.init_node('puCSV2Topic_allignGPSTime', anonymous=True)
    csv2Topi_ = csv2Topi()
    # csv2Topi_.fastReadCSV(485204000,485422000) # the value is provided by bag file in topci '/novatel_data/bestpos' # for 
    # csv2Topi_.fastReadCSV(47743000,48135000) # the value is provided by bag file in topci '/novatel_data/bestpos'
    # csv2Topi_.fastReadCSV(376657000,376975000) # the value is provided by bag file in topci '/novatel_data/bestpos'
    # 204102000
    # csv2Topi_.fastReadCSV(204102000,204250000) # inside gnss small loop 1
    # csv2Topi_.fastReadCSV(204255000,204429000) # inside gnss small loop 2

    csv2Topi_.fastReadCSV(204437000,204801000) # inside gnss large loop 1
    # csv2Topi_.fastReadCSV(204824000,205131000) # inside gnss large loop 2
    rate = rospy.Rate(0.01)  # 1hz
    while not rospy.is_shutdown():
        hello_str = "current time is %s" % rospy.get_time()
        rate.sleep()