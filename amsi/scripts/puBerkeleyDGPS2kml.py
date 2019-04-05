#! /usr/bin/env python
# -*- coding=utf-8 -*-
# finished by Dr. WEN
"""
    Function: subscribe GNSS related rostopic and save it into a KML file
    Welson Wen, Ph.D.
    https://sites.google.com/view/weisongwen/about-me

    subcribe : 'gnss_navsat' and 'gnss_ins_navsat'
    note: those two topic is published by the amsi_nodelet by subscribing the NMEA sentence from DGPS

"""
from lxml import etree 
import xlrd             
import rospy
from pykml.factory import KML_ElementMaker as KML 
import csv # csv reading needed library
import pandas as pd
from novatel_msgs.msg import BESTPOS
from sensor_msgs.msg   import NavSatFix # standard message type for GNSSs


class pullh2kml_eval():
    def __init__(self):
        rospy.Subscriber('/gnss_navsat', NavSatFix, self.callgnss_navsat)
        rospy.Subscriber('/gnss_ins_navsat', NavSatFix, self.callgnss_ins_navsat)
        self.lat_ = [] # used to save latitude
        self.lon_ = [] # used to save longitude
        self.ukf_lat_ = []
        self.ukf_lon_ =[]
        self.writeToKML = 0.0

    def callgnss_navsat(self,data):
        self.bestPos_ = NavSatFix()
        self.bestPos_ = data
        self.lat_.append(float(self.bestPos_.latitude))
        self.lon_.append(float(self.bestPos_.longitude))

        fold = KML.Folder(KML.Placemark(KML.Point(KML.coordinates(str(self.lon_[0]) + ',' + str(self.lat_[0]) + ',0'))))
        for i in range(1, len(self.lon_)):
            fold.append(KML.Placemark(
                KML.Point(KML.coordinates(str(self.lon_[i]) + ',' + str(self.lat_[i]) + ',0'))))
        content = etree.tostring(etree.ElementTree(fold), pretty_print=True)
        with open('Berkeley_gnss.kml', 'w') as fp:
            fp.write(content)

    def callgnss_ins_navsat(self,data):
        self.uck_bestPos_ = NavSatFix()
        self.uck_bestPos_ = data
        self.ukf_lat_.append(float(self.uck_bestPos_.latitude))
        self.ukf_lon_.append(float(self.uck_bestPos_.longitude))
        fold = KML.Folder(KML.Placemark(KML.Point(KML.coordinates(str(self.ukf_lon_[0]) + ',' + str(self.ukf_lat_[0]) + ',0'))))
        for i in range(1, len(self.ukf_lon_)):
            fold.append(KML.Placemark(
                KML.Point(KML.coordinates(str(self.ukf_lon_[i]) + ',' + str(self.ukf_lat_[i]) + ',0'))))
        content = etree.tostring(etree.ElementTree(fold), pretty_print=True)
        with open('Berkeley_gnss_ins.kml', 'w') as fp:
            fp.write(content)


if __name__ == '__main__':
    rospy.init_node('pullh2kml_evaluGt', anonymous=True)
    pullh2kml_eval_ =pullh2kml_eval()
    rate = rospy.Rate(0.002)#
    preTim = 0.0
    while not rospy.is_shutdown():
        rate.sleep()
        print 'sleep end '