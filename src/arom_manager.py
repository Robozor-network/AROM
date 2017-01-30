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
import pylirc
from drivers.__init__ import AromNode

try:
    import xml.etree.cElementTree as ET
except ImportError:
    import xml.etree.ElementTree as ET

recieved = []

def manager_callback(recive):
    global recieved
    recieved.append(recive.data)
    #print recive, recieved

class arom_manager(AromNode):
    node_name = "arom_manager"
    node_type = "arom_manager"
    node_pymlab = True

    def __init__(self, rows = 2, cols = 2, file = None):
        print os.path.abspath(__file__)

        rospy.Subscriber("/arom/manager", std_msgs.msg.String, manager_callback)

        AromNode.__init__(self)
        self.set_feature('arom_nodes',{'publish': '/arom/manager'})

        ##
        ##  Konec zakladni inicializace
        ##

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                if len(recieved) > 0:
                    print recieved
            except Exception, e:
                print e
            time.sleep(0.5)

   


if __name__ == '__main__':
    m = arom_manager()
