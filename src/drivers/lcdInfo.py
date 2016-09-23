#!/usr/bin/env python

import math
import time
import rospy
import std_msgs
import json
from std_msgs.msg import String
from std_msgs.msg import Float32
from arom.srv import *
from arom.msg import *
import numpy as np
import pylirc

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

class lcdInfo16x2(object):
    def __init__(self, rows = 2, cols = 2, file = None):
        self.rows = rows
        self.cols = cols
        self.loadCFG(file)
        self.menu_row_id = 0
        self.menu_row_path = "./"
        self.runningApp = None

        rospy.Subscriber("/aws_out", msg_WeatherStation, callback_aws)
        rospy.Subscriber("/arom/UI/buttons", String, callback_btn)
        self.pymlab = rospy.ServiceProxy('pymlab_drive', PymlabDrive)
        rospy.init_node('lcd_info_1620')
        print "zinicializovano"

        self.pymlab(device="StatusLCD",   method="reset")
        self.pymlab(device="StatusLCD",   method="init")
        self.pymlab(device="StatusLCD",   method="clear")
        self.pymlab(device="StatusLCD",   method="home")
        
        time.sleep(0.5)
        self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineA="Welcome...", lineB="AROM     MLAB.cz")))
        
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
        print "### enter", self.menu_selected_row.tag
        if self.menu_selected_row.tag == 'folder':
            print "select menu", self.menu_selected_row
            self.menu_row_path += "folder[@text='%s']/" %(self.menu_selected_row.attrib['text'])
            print self.menu_row_path
            print self.menu_selected_row
            self.menu_row_id = 0
            self.display()
        elif self.menu_selected_row.tag == 'widget':
            print "widget", self.menu_selected_row.attrib
            self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineB="Loading ...", lineA="widget")))
            time.sleep(0.25)
            #self.runningApp = self.menu_selected_row.attrib['function']
            getattr(self, self.menu_selected_row.attrib['function'])()

        elif self.menu_selected_row.tag == 'attrib':
            print "blablablabla"
            self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineB="ahoj :)", lineA="Super !!!")))

        else:
            self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineB=str("Err 001"), lineA="AROM     Error")))


    def back(self):
        self.menu_row_path = "./"
        self.display()

    def display(self):
        try:
            out = self.root.findall(self.menu_row_path)
            print "#", out[self.menu_row_id].tag, out[self.menu_row_id].attrib
            self.menu_selected_row = out[self.menu_row_id]
            if self.menu_selected_row.tag == 'folder':
                text = '.' + out[self.menu_row_id].attrib['text']
            elif self.menu_selected_row.tag == 'widget':
                text = '>' + out[self.menu_row_id].attrib['text']
            #id = str(self.menu_row_id)
            lineB = text#.ljust(15-len(id))+" "+id
            self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineB=lineB, lineA="AROM  MLAB.cz"+" "+str(self.menu_row_id))))

        except Exception, e:
            rospy.logerr(e)
            self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineB=str(e), lineA="AROM     Error")))


    def loadCFG(self, file = None):
        if file:
            self.cfg = file
        self.tree = ET.parse(self.cfg) 


    def getNetwork(self):
        self.runningApp = 'getNetwork'

        import netifaces as ni
        ip = ni.ifaddresses('eth0')[2][0]['addr']
        self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineB=str(ip), lineA="AROM     Network")))


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
                self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineB=str(e), lineA="AROM     Error")))



    def getTime(self):
        self.runningApp = 'getTime'
        i = 0

        self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineB=str(ip)), lineA="AROM     Network"))

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
                self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineB=str(e), lineA="AROM     Error")))

    def ShowDateTime(self):
        self.runningApp = 'ShowDateTime'
        i = 0
        while not rospy.is_shutdown() and self.runningApp == 'ShowDateTime':
            try:
                time.sleep(0.5)
                self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineB=time.strftime("%Y-%m-%d", time.gmtime()), lineA=str(time.strftime("%H:%M:%S UT", time.gmtime())) )))
            except Exception, e:
                rospy.logerr(e)
                self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineB=str(e), lineA="AROM     Error")))

            if len(btn_data) > 0:
                lastBtn = btn_data[0]
                btn_data.pop(0)
                if lastBtn == 'KEY_F2':  # Back
                    self.runningApp = None
                    self.display()



    def getWeather(self):
        index = 0
        param  = list(aws_data)[index]
        self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineB="%.4f" %(aws_data[param]), lineA=param)))
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
                        self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineB="%.4f" %(aws_data[param]), lineA=param)))


                    elif lastBtn == 'KEY_VOLUMEDOWN':
                        if index < 0:
                            index = 0
                        index -= 1
                        param  = list(aws_data)[index]
                        self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineB="%.4f" %(aws_data[param]), lineA=param)))


                    elif lastBtn == 'KEY_F3': # OK
                        param  = list(aws_data)[index]
                        print param
                        self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineB="%.4f" %(aws_data[param]), lineA=param)))

                    elif lastBtn == 'KEY_F2':  # Back
                        self.runningApp = None
                        self.display()
                if i > 100:
                    i = 0
                    self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineB="%.4f" %(aws_data[param]), lineA=param)))
                i += 1

            except Exception, e:
                rospy.logerr(e)
                self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineB=str(e), lineA="AROM     Error")))


if __name__ == '__main__':
    m = lcdInfo16x2(file='/home/odroid/rosws/src/AROM/cfg/lcdAROM.xml')
    #m.loadCFG('/home/odroid/rosws/src/AROM/cfg/lcdAROM.xml')
