#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
from arom.srv import *
from arom.msg import *

class AromNode():
    def __init__(self):

        if self.node_pymlab:
            self.pymlabService = rospy.ServiceProxy('pymlab_drive', PymlabDrive)

        print "Starting init"
        rospy.init_node(self.node_name)
        rospy.set_param('/arom/node'+rospy.get_name()+"/type", self.node_type)
        self.node_name = rospy.get_name()
        print "Init done:", rospy.get_name()

    def pymlab(self, *args, **kwds):
        self.pymlabService(**kwds)

class Pymlab():
    def __init__(self):
        self.serv = rospy.ServiceProxy('pymlab_drive', PymlabDrive)

    def send(self, device=None, method=None, parameters=None):
        return self.serv(device=device, method=method, parameters=parameters)