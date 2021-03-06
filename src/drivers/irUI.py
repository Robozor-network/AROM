#!/usr/bin/env python
# -*- coding: utf-8 -*-

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




class irUI(object):
    def __init__(self, name = "irUI"):
        self.pub_btn = rospy.Publisher('/arom/UI/buttons', String, queue_size=10)
        self.pub_beep = rospy.Publisher('/beeper/preset', String, queue_size=5)

        rospy.init_node('irUI')
        self.set_feature('remoteCtrl',{'type': "odroid"})
        blocking = 0;

        rate = rospy.Rate(10)
        pylirc.init("pylirc", "/home/odroid/robozor/station/irConf.conf", blocking)
        while not rospy.is_shutdown():
            try:
                s = pylirc.nextcode(1)
                if s:
                    rospy.loginfo(s[0]['config'])
                    self.pub_btn.publish(s[0]['config'])
                    self.pub_beep.publish('btn')
                s = []

            except Exception, e:
                rospy.logerr(e)
            rate.sleep()

        pylirc.exit()
        self.connection.close()


if __name__ == '__main__':
    m = irUI()