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
import serial
from astropy.time import Time


class weatherStation(object):
    def __init__(self, parent = None, arg = None, name = "AWS01B", port="", connect = True, var = {}):
        self.arg = arg
        self.Autoconnect = connect
        self.port = port
        self.parent = parent
        self.name = name
        self.variables = var

        ##
        ##  Inicializace vlastniho ovladace
        ##

        self.init()

        s_RegisterDriver = rospy.Service('driver/weatherStation/%s' %self.name, arom.srv.DriverControl, self.reset)

        ##
        ##  Ceka to na spusteni AROMbrain nodu
        ##

        rospy.init_node('AROM_weatherStation')
        rospy.loginfo("%s: wait_for_service: 'arom/RegisterDriver'" % self.name)
        rospy.wait_for_service('arom/RegisterDriver')
        rospy.loginfo("%s: >> brain found" % self.name)

        ##
        ##  Registrace zarizeni
        ##  >Arom returns 1 - OK, 0 - False
        ##

        RegisterDriver = rospy.ServiceProxy('arom/RegisterDriver', arom.srv.RegisterDriver)
        registred = RegisterDriver(name = self.name, sname= self.name, driver = 'AWS01A', device = 'weatherStation', status = 1)
        rospy.loginfo("%s: >> register %s driver: %s" %(self.name, 'AWS01A', registred))


        ##
        ##  Ovladac se pripoji k montazi
        ##

        if self.Autoconnect:
            self.connect()

        ##
        ##  Ovladac pujde ukoncit
        ##

        rare = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            self.mesure()
            rare.sleep()


    def reset(self, val=None):
        pass

############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################

######################################################################################
######################################################################################
##                                                                                  ##
##                  Driver for --AWS01A-- MLAB weather station                      ##
##                 ============================================                     ##
##                                                                                  ##
##                                                                                  ##
######################################################################################
        
class AWS01B(weatherStation):
    def init(self): 
        self.variables = {
            'AWS_LTS_temp': 0,
            'AWS_SHT_temp': 0,
            'AWS_SHT_humi': 0
            }
        rospy.loginfo("AWS01A weather station requires 'pymlab_drive' service from 'ROSpymlabServer' node")
        rospy.loginfo("run>> 'rosrun arom initPymlab.py'")
        rospy.wait_for_service('pymlab_drive')
        self.pymlab = rospy.ServiceProxy('pymlab_drive', PymlabDrive)
        rospy.loginfo("%s: >> 'ROSpymlabServer' found" % self.name)


    def mesure(self):
        self.variables['AWS_LTS_temp'] = self.pymlab(device="AWS_temp", method="get_temp", parameters=None).value
        self.variables['AWS_SHT_temp'] = self.pymlab(device="AWS_humi", method="get_temp_8bit", parameters=None).value
        self.variables['AWS_SHT_humi'] = self.pymlab(device="AWS_humi", method="get_hum_8bit", parameters=None).value
        print self.variables
        rospy.set_param("weatherStation", str(self.variables))
        

    def connect(self):
        pass



if __name__ == '__main__':
    cfg = rospy.get_param("ObservatoryConfig/file")
    with open(cfg) as data_file:
        config = json.load(data_file)
    for x in config:
        if x['name'] == sys.argv[1]:
            break
    weatherStation = locals()[x['driver']](arg = x)
    #weatherStation = AWS01B()