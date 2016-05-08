#!/usr/bin/env python

import sys
import rospy
import pymlab
from pymlab import config
import std_msgs
from std_msgs.msg import String
from std_msgs.msg import Float32
from arom.srv import *
from arom.msg import *

import json
import drivers
#from drivers import devices
#from drivers import camera
#from drivers import focuser
##from drivers import weatherStation
#from drivers import rotator
#from drivers import roof
#import drivers


class AromBrain():

    def __init__(self):
        rospy.init_node('AROM_brain')
        self.mount = {}
        self.weatherStation = {}
        self.camera = {}
        self.devices = {}

        s_RegisterDriver = rospy.Service('/arom/RegisterDriver', arom.srv.RegisterDriver, self.RegisterDriver)
        s_NodeInfo = rospy.Service('/arom/NodeInfo', arom.srv.NodeInfo, self.NodeInfo)

        self.config_file = sys.argv[1].decode('utf-8')
        with open(self.config_file) as data_file:    
            self.config = json.load(data_file)
        rospy.set_param("ObservatoryConfig/file", str(self.config_file))
        rospy.loginfo(str(self.config_file))
        
        rospy.loginfo("AROMbrain started")
        rospy.spin()

    def RegisterDriver(self, srv):
        rospy.loginfo("NewDevice>> type: %s, name %s (%s). With driver %s" %(srv.device, srv.name, srv.sname, srv.driver))
        self.devices[srv.sname] = {'name': srv.name, 'driver': srv.driver, 'device': srv.device, 'service': srv.service}
        print self.devices
        return 1

    def NodeInfo(self, srv):
        print srv
##  Ziska parametry nodu dle jmena
        if srv.type == "GetNode":
            return arom.srv.NodeInfoResponse(data = repr(self.devices['srv.data']), done = True)

##  Ziska seznam vsech nodu s parametrama
        elif srv.type == "GetAllNodes":
            print self.devices
            return arom.srv.NodeInfoResponse(data = repr(self.devices), done = True)

##  Ziska seznam nodu podle device .. napr podle mount, camera, ...
        elif srv.type == "GetDeviceNode":
            rospy.logerr("Error: NotImplemented: %s" %(srv.type))
            return arom.srv.NodeInfoResponse(data = repr("Error: NotImplemented"), done = True)

        else:
            rospy.logerr("Error: Unknown data type in 'srv.NodeInfo': %s" %(srv.type))
            return arom.srv.NodeInfoResponse(data = repr("Error: Unknown data type in 'srv.NodeInfo'"), done = True)


    def loadDriver(self, deviceType = None, driverName = 'mount'):
        try:
            driver = None
            if deviceType:
                driver = getattr(eval('mount'), driverName)(self)
        except Exception, e:
            print "Error:", e
        finally:
            return driver

    def mount_move(self, target = [10,10]):
        print 
                    


def main():
    ab = AromBrain()

if __name__ == "__main__":
    main()