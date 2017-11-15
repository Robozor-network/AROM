#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import arom
from arom import srv 

class AromNode():
    def __init__(self):

        try:
            print "AromNode INIT"
            if self.node_pymlab:
                self.pymlab = rospy.ServiceProxy('pymlab_drive', srv.PymlabDrive)
                rospy.set_param('/arom/node'+rospy.get_name()+"/pymlab", True)
            else:
                rospy.set_param('/arom/node'+rospy.get_name()+"/pymlab", False)
                print "Chyba pri vytvareni pymlabu"
                rospy.logerr("Chyba pri vytvareni pymlabu")
        except Exception, e:
            print e
            rospy.set_param('/arom/node'+rospy.get_name()+"/pymlab", False)

        print "Starting init"
        rospy.init_node(self.node_name)
        rospy.set_param('/arom/node'+rospy.get_name()+"/type", self.node_type)
        self.node_name = rospy.get_name()
        print "Init done:", rospy.get_name()

    #def pymlab(self, *args, **kwds):
        #self.pymlabService(**kwds)

    def set_feature(self, name, value):
        rospy.set_param('/arom/node%s/feature/%s' %(str(rospy.get_name()),name), value)
        rospy.loginfo("New feature '%s' for %s" %(rospy.get_name(), name))