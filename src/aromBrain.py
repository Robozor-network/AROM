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
from drivers import mount
from drivers import camera
from drivers import focuser
from drivers import weatherStation
from drivers import rotator
from drivers import roof
#import drivers


class AromBrain():
    def __init__(self):
        rospy.init_node('AROM_brain')
        self.drivers = {}

        s_RegisterDriver = rospy.Service('arom/RegisterDriver', arom.srv.RegisterDriver, self.RegisterDriver)

        self.config_file = sys.argv[1].decode('utf-8')
        with open(self.config_file) as data_file:    
            self.config = json.load(data_file)
        rospy.set_param("ObservatoryConfig/file", str(self.config_file))


        rospy.spin()

    def RegisterDriver(self, srv):
        print "RegisterDriver:", srv
        self.drivers[srv.sname] = srv
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
                    


def main():

    #rospy.Subscriber("pymlab_server", PymlabServerStatusM, ps.status)
    #s1 = rospy.Service('pymlab_init', PymlabInit, ps.init)
    #s2 = rospy.Service('pymlab_server', PymlabServerStatus, ps.status)
    #s3 = rospy.Service('pymlab_drive', PymlabDrive, ps.drive)

    #rospy.loginfo("Ready to get work.")
    #rospy.spin()

    ab = AromBrain()
    #ab.load()

if __name__ == "__main__":
    main()