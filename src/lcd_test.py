#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import time
import rospy
import rosnode
import std_msgs
import sensor_msgs
import geometry_msgs
from geometry_msgs.msg import Point
import arom
import axis
import json
import os
import arom_helper

class LCD(arom_helper.AromNode):
    node_name = "LCD"
    node_type = "LCD"
    def __init__(self):
        arom_helper.AromNode.__init__(self)
        lcd  = rospy.Publisher('/arom/node/lcdText', std_msgs.msg.String, queue_size=10)
        lcd.publish("LCD test")
        print "Test done"
        while not rospy.is_shutdown():
            pass
        
lcd = LCD()