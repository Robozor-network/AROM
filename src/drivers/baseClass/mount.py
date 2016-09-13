#!/usr/bin/env python

import math
import time
import rospy
import std_msgs
import actionlib
import json
from std_msgs.msg import String
from std_msgs.msg import Float32
from arom.srv import *
from arom.msg import *
import numpy as np

from pydirectmount.drive import drive

btn_data = []

def callback(recive):
    #for i, type in enumerate(recive.type):
    #    self.data[type] = recive.value[i]
    print recive

def callback_btn(recive):
    global btn_data
    btn_data.append(recive)
    print recive, btn_data

class mount(object):
    def __init__(self, parent = None, arg = None, name = "mount", port="", connect = True, var = {}):
        self.arg = arg
        self.Autoconnect = connect
        self.port = port
        self.parent = parent
        self.name = name
        self.sname = self.name
        self.variables = var
        self.rate = 1

        self.callback_btn = ()

        #self.init()

        rospy.Subscriber("/mount/controll", String, callback)
        rospy.Subscriber("/arom/UI/buttons", String, callback_btn)
        self.pub_status = rospy.Publisher('/mount/status', String, queue_size=10)

        rospy.init_node('AROM_mount')

        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            try:
                print "-", self.callback_btn
            except Exception, e:
                rospy.logerr(e)
            rate.sleep()


        self.connection.close()

        '''
        def callback(self, recive):
            #for i, type in enumerate(recive.type):
            #    self.data[type] = recive.value[i]
            print recive
        
        def callback_btn(self, recive):
            self.btn_data.append(recive)
            print recive, self.btn_data
        '''


if __name__ == '__main__':
    m = mount()