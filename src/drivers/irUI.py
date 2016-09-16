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




class irUI(object):
    def __init__(self, name = "irUI"):
        self.pub_weather = rospy.Publisher('/arom/UI/buttons', String, queue_size=10)

        rospy.init_node('irUI')
        blocking = 0;

        rate = rospy.Rate(10)
        pylirc.init("pylirc", "/home/odroid/rosws/src/AROM/cfg/irConf", blocking)
        while not rospy.is_shutdown():
            try:
                s = pylirc.nextcode(1)
                if s:
                    rospy.loginfo(s[0]['config'])
                    self.pub_weather.publish(s[0]['config'])
                s = []

            except Exception, e:
                rospy.logerr(e)
            rate.sleep()

        pylirc.exit()
        self.connection.close()


if __name__ == '__main__':
    m = irUI()