#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time
import rospy
import std_msgs
import json
import os
from std_msgs.msg import String
from std_msgs.msg import Float32
from arom.srv import *
from arom.msg import *
import numpy as np
from __init__ import AromNode

try:
    import xml.etree.cElementTree as ET
except ImportError:
    import xml.etree.ElementTree as ET

aws_data = {}
btn_data = []

def callback_aws(recive):
    global aws_data
    for i, type in enumerate(recive.type):
        aws_data[type] = recive.value[i]
    #print aws_data

def callback_btn(recive):
    global btn_data
    btn_data.append(recive.data)
    #print recive, btn_data

class lcdInfo16x2(AromNode):
    node_name = "lcdInfo16x2"
    node_type = "lcd_text"
    node_pymlab = True

    def __init__(self, rows = 2, cols = 2, file = None):
        print os.path.abspath(__file__)

        rospy.Subscriber("/aws_out", msg_WeatherStation, callback_aws)
        rospy.Subscriber("/arom/UI/buttons", String, callback_btn)
        self.lcdText_pub = rospy.Publisher('/arom/node/lcdText', std_msgs.msg.String, queue_size=10)


        AromNode.__init__(self)
        self.set_feature('display_show',{'rows': rows, 'cols': cols, 'publish': '/arom/node/lcdText'})
        self.set_feature('display_control', {'up': 'KEY_VOLUMEUP', 'down': 'KEY_VOLUMEDOWN', 'back': 'KEY_F2', 'enter': 'KEY_F3', 'subscrib': '/arom/UI/buttons'})

        ##
        ##  Konec zakladni inicializace
        ##

        self.rows = rows
        self.cols = cols
        self.loadCFG(file)
        self.menu_row_id = 0
        self.menu_row_path = "./"
        self.runningApp = None

        print "zinicializovano"

        self.pymlab(device="StatusLCD",   method="reset")
        self.pymlab(device="StatusLCD",   method="init")
        #self.pymlab(device="StatusLCD",   method="clear")
        self.pymlab(device="StatusLCD",   method="home")

        time.sleep(0.5)
        self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineA="Welcome...", lineB="AROM    MLAB.cz")))

        print "-------------"
        self.root = self.tree.getroot()


        rate = rospy.Rate(10)
        ra = 0
        dec = 90
        while not rospy.is_shutdown():
            try:
                if len(btn_data) > 0 and self.runningApp == None:
                    print btn_data[0], len(btn_data)
                    lastBtn = btn_data[0]
                    btn_data.pop(0)

                    print lastBtn
                    if lastBtn == 'KEY_VOLUMEUP':
                        self.up()

                    elif lastBtn == 'KEY_VOLUMEDOWN':
                        self.down()

                    elif lastBtn == 'KEY_F3': # OK
                        self.enter()

                    elif lastBtn == 'KEY_F2':  # Back
                        self.back()

            except Exception, e:
                print e
            time.sleep(0.1)

    def setLCD(self, lineA, lineB):
        self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineA=str(lineA), lineB=str(lineB))))
        self.lcdText_pub.publish(str(lineA)+"\n"+str(lineB))


    def up(self):
        self.menu_row_id += 1
        self.display()

    def down(self):
        if self.menu_row_id > 0:
            self.menu_row_id -= 1
        else:
            self.menu_row_id = 0
        self.display()

    def enter(self):
        if self.menu_selected_row.tag == 'folder':
            self.menu_row_path += "folder[@text='%s']/" %(self.menu_selected_row.attrib['text'])
            self.menu_row_id = 0
            self.display()
        elif self.menu_selected_row.tag == 'widget':
            self.setLCD(lineB="Loading ...", lineA="widget")
            time.sleep(0.25)
            getattr(self, self.menu_selected_row.attrib['function'])()

        elif self.menu_selected_row.tag == 'attrib':
            self.setLCD(lineB="ahoj :)", lineA="Super !!!")

        else:
            self.setLCD(lineB=str("Err 001"), lineA="AROM     Error")


    def back(self):
        self.menu_row_path = "./"
        self.display()

    def display(self):
        try:
            out = self.root.findall(self.menu_row_path)
            self.menu_selected_row = out[self.menu_row_id]
            if self.menu_selected_row.tag == 'folder':
                text = '.' + out[self.menu_row_id].attrib['text']
            elif self.menu_selected_row.tag == 'widget':
                text = '>' + out[self.menu_row_id].attrib['text']
            self.setLCD(lineB=text, lineA="AROM  MLAB.cz"+" "+str(self.menu_row_id))

        except Exception, e:
            rospy.logerr(e)
            self.setLCD(lineB=str(e), lineA="AROM     Error")


    def loadCFG(self, file = None):
        if file:
            self.cfg = file
        self.tree = ET.parse(self.cfg) 


    def getNetwork(self):
        self.runningApp = 'getNetwork'

        import netifaces as ni
        ip = ni.ifaddresses('eth0')[2][0]['addr']
        self.setLCD(lineB=str(ip), lineA="AROM     Network")


        while not rospy.is_shutdown() and self.runningApp == 'getNetwork':
            try:
                time.sleep(0.3)
                if len(btn_data) > 0:
                    lastBtn = btn_data[0]
                    btn_data.pop(0)

                    if lastBtn == 'KEY_F2':  # Back
                        self.runningApp = None
                        self.display()

            except Exception, e:
                rospy.logerr(e)
                self.setLCD(lineB=str(e), lineA="AROM     Error")



    def getTime(self):
        self.runningApp = 'getTime'
        i = 0

        self.setLCD(lineB=str(ip), lineA="AROM     Network")

        while not rospy.is_shutdown() and self.runningApp == 'getNetwork':
            try:
                time.sleep(0.3)
                if len(btn_data) > 0:
                    lastBtn = btn_data[0]
                    btn_data.pop(0)

                    if lastBtn == 'KEY_F2':  # Back
                        self.runningApp = None
                        self.display()

            except Exception, e:
                rospy.logerr(e)
                self.setLCD(lineB=str(e), lineA="AROM     Error")

    def ShowDateTime(self):
        self.runningApp = 'ShowDateTime'
        i = 0
        while not rospy.is_shutdown() and self.runningApp == 'ShowDateTime':
            try:
                time.sleep(0.5)
                self.setLCD(lineB=time.strftime("%Y-%m-%d", time.gmtime()), lineA=str(time.strftime("%H:%M:%S UT", time.gmtime())))
            except Exception, e:
                rospy.logerr(e)
                self.setLCD(lineB=str(e), lineA="AROM     Error")

            if len(btn_data) > 0:
                lastBtn = btn_data[0]
                btn_data.pop(0)
                if lastBtn == 'KEY_F2':  # Back
                    self.runningApp = None
                    self.display()



    def getWeather(self):
        index = 0
        param  = list(aws_data)[index]
        self.setLCD(lineB="%.4f" %(aws_data[param]), lineA=param)
        self.runningApp = 'getWeather'
        i = 0

        while not rospy.is_shutdown() and self.runningApp == 'getWeather':
            try:
                time.sleep(0.1)
                if len(btn_data) > 0 and self.runningApp == 'getWeather':
                    lastBtn = btn_data[0]
                    btn_data.pop(0)

                    print lastBtn
                    if lastBtn == 'KEY_VOLUMEUP':
                        if index > len(aws_data):
                            index = len(aws_data)
                        index += 1
                        param  = list(aws_data)[index]
                        self.setLCD(lineB="%.4f" %(aws_data[param]), lineA=param)


                    elif lastBtn == 'KEY_VOLUMEDOWN':
                        if index < 0:
                            index = 0
                        index -= 1
                        param  = list(aws_data)[index]
                        self.setLCD(lineB="%.4f" %(aws_data[param]), lineA=param)
    

                    elif lastBtn == 'KEY_F3': # OK
                        param  = list(aws_data)[index]
                        print param
                        self.setLCD(lineB="%.4f" %(aws_data[param]), lineA=param)

                    elif lastBtn == 'KEY_F2':  # Back
                        self.runningApp = None
                        self.display()
                if i > 100:
                    i = 0
                    self.setLCD(lineB="%.4f" %(aws_data[param]), lineA=param)
                i += 1

            except Exception, e:
                rospy.logerr(e)
                self.setLCD(lineB=str(e), lineA="AROM     Error")


if __name__ == '__main__':
    m = lcdInfo16x2(file='/home/odroid/ros_ws/src/AROM/cfg/lcdAROM.xml')
    #m.loadCFG('/home/odroid/rosws/src/AROM/cfg/lcdAROM.xml')
