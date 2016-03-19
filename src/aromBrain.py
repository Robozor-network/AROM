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
from drivers import weatherStation
#from drivers import rotator
#from drivers import roof
#import drivers


class AromBrain():
    def mountSlew(self, cor = [0,0]):
        pass

    def __init__(self):
        rospy.init_node('AROM_brain')
        self.mount = {}
        self.weatherStation = {}
        self.camera = {}
        self.devices = {}

        #from drivers import mount, camera, roof, weatherStation, focuser, rotator
       # self.drivers = {
        #    'EQmod': mount.EQmod,
        #    'SynScan': mount.SynScan,
        #    'mount': mount.EQmod,
        #    'AWS01B': weatherStation.AWS01B,
        #}

        s_RegisterDriver = rospy.Service('arom/RegisterDriver', arom.srv.RegisterDriver, self.RegisterDriver)

        self.config_file = sys.argv[1].decode('utf-8')
        with open(self.config_file) as data_file:    
            self.config = json.load(data_file)
        rospy.set_param("ObservatoryConfig/file", str(self.config_file))
        rospy.loginfo(str(self.config_file))
        
        rospy.loginfo("AROMbrain started")
        rospy.spin()

    def RegisterDriver(self, srv):
        #dev_driver = self.drivers[srv.driver]
        #eval('self.'+srv.device)[srv.sname] = {'name':srv.name, 'sname':srv.sname, 'driver':srv.driver, 'object':dev_driver}
        #self.devices[srv.sname] = {'name':srv.name, 'sname':srv.sname, 'driver':srv.driver, 'object':dev_driver}
        
        rospy.loginfo("NewDevice>> type: %s, name %s (%s). With driver %s" %(srv.device, srv.name, srv.sname, srv.driver))
        return 1

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