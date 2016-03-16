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


class UserInteface(object):
    def __init__(self, parent = None, arg = None, name = "I2CLCD", port="", connect = True, var = {}):
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

        s_RegisterDriver = rospy.Service('driver/userInteface/%s' %self.name, arom.srv.DriverControl, self.reset)

        ##
        ##  Ceka to na spusteni AROMbrain nodu
        ##

        rospy.init_node('AROM_userInterface')
        rospy.loginfo("%s: wait_for_service: 'arom/RegisterDriver'" % self.name)
        rospy.wait_for_service('arom/RegisterDriver')
        rospy.loginfo("%s: >> brain found" % self.name)

        ##
        ##  Registrace zarizeni
        ##  >Arom returns 1 - OK, 0 - False
        ##

        RegisterDriver = rospy.ServiceProxy('arom/RegisterDriver', arom.srv.RegisterDriver)
        registred = RegisterDriver(name = self.name, sname= self.name, driver = 'I2CLCD', device = 'userInteface', status = 1)
        rospy.loginfo("%s: >> register %s driver: %s" %(self.name, 'I2CLCD', registred))


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
            self.run()
            rare.sleep()

    def run(self):
    	pass

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
        
class i2clcd(UserInteface):
    def init(self): 
        
        rospy.loginfo("I2CLCD user inteface requires 'pymlab_drive' service from 'ROSpymlabServer' node")
        rospy.loginfo("run>> 'rosrun arom initPymlab.py'")
        rospy.wait_for_service('pymlab_drive')
        self.pymlab = rospy.ServiceProxy('pymlab_drive', PymlabDrive)
        rospy.loginfo("%s: >> 'ROSpymlabServer' found" % self.name)
        
    def run(self):
    	weather = rospy.get_param("weatherStation")
    	print weather


        self.pymlab(device="StatusLCD", method="set_clear", parameters=None)
        time.sleep(0.05)
        self.pymlab(device="StatusLCD", method="set_home", parameters=None)
        time.sleep(0.05)
        self.pymlab(device="StatusLCD", method="puts", parameters="Tem:%.2fC SHT25" % (i, temperature))
        time.sleep(0.05)
        self.pymlab(device="StatusLCD", method="set_row2", parameters=None)
        time.sleep(0.05)
        self.pymlab(device="StatusLCD", method="puts", parameters="%dTem:%.2fC SHT25" % (i, temperature))


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