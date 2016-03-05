#!/usr/bin/env python

import math
import time
import rospy
import std_msgs
import actionlib
from std_msgs.msg import String
from std_msgs.msg import Float32
from arom.srv import *
from arom.msg import *
import arom
import serial


class mount():
    def __init__(self, parent = None, name = "mount", port="/dev/ttyUSB0", connect = 1):
        self.Autoconnect = connect
        self.port = port
        self.parent = parent
        self.name = name
        self.coordinates = [0,0]  ## Ra, Dec
        self.coordinates_geo = [0, 0, 0] ## Lat, Lon, Alt

        ##
        ##  Ceka to na spusteni AROMbrain nodu
        ##

        rospy.init_node('AROM_mount')
        rospy.loginfo("%s: wait_for_service: 'arom/RegisterDriver'" % self.name)
        rospy.wait_for_service('arom/RegisterDriver')
        rospy.loginfo("%s: >> done" % self.name)

        ##
        ##  Registrace zarizeni
        ##  >Arom returns 1 - OK, 0 - False
        ##

        RegisterDriver = rospy.ServiceProxy('arom/RegisterDriver', arom.srv.RegisterDriver)
        registred = RegisterDriver(name = self.name, driver = 'SynScan', status = 1)
        rospy.loginfo("%s: >> register %s" %(self.name, registred))

        ##
        ##  Spusti se Action servis pro zmenu cile
        ##

        self.act = actionlib.SimpleActionServer('AROM/mount/target', arom.msg.MountTargetAction, execute_cb=self.ReciveTarget, auto_start = False)
        self.act.start()

        ##
        ##  Ovladac se pripoji k montazi
        ##

        if self.Autoconnect:
            self.connect()

        ##
        ##  Vytvoreni servisu na praci s montazi
        ##

        self.s_MountParameters = rospy.Service('arom/mount/parameter', arom.srv.MountParameter, self.MountParameter)

        ##
        ##  Ovladac pujde ukoncit
        ##

        rospy.spin()


    def MountParameter(self, MountParameter = None):
        rospy.loginfo('%s: GetNewParameters: %s' %(self.name, MountParameter))
        out = getattr(self, str(MountParameter.name))()
        rospy.loginfo('%s: GetNewParameters: out >> %s' %(self.name, str(out)))
        done = {'value': str(out), 'parameters': '', 'done': 1}
        print done
        return arom.srv.MountParameterResponse(str(out), str('raw'), 1)

    def ReciveTarget(self, target = None):
        print "Target:", target
        rospy.loginfo('%s: GetNewTarget: %s' %(self.name, target))

    def reciveMSG(self, msg):
        pass

    def connect(self):
        print "connecting"

    def park(self):
        print "park"
        
    def unpark(self):
        print "UNpark"
        
    def setpark(self):
        print "SENpark"
        
    def getpark(self):
        print "GENpark"

    def start(self):
        print "EQmod driver started"
        
    def slew(self):
        raise NotImplementedError()

    def track(self):
        raise NotImplementedError()

    def setPosition(self):
        raise NotImplementedError()

    def getPosition(self, param = None):
        print "position :)", param

    def getStatus(self):
        raise NotImplementedError()

    def setAligmentPoint(self):
        raise NotImplementedError()

    def getAligmentPoint(self):
        raise NotImplementedError()

    def setLimits(self):
        raise NotImplementedError

    def getLimits(self):
        raise NotImplementedError()

    def getDriverVer(self):
        print "version is blablabla"
        

class EQmod(mount):
    def __init__(self, parent=None):
        print "init V EQmod"

    def start(self):
        print "EQmod driver started"
        

######################################################################################
######################################################################################
##                                                                                  ##
##                   Driver for SynScan mount hand controller                       ##
##                 ============================================                     ##
##                                                                                  ##
##      It does not support telescope setAligmentPoint                              ##
##                                                                                  ##
######################################################################################

class SynScan(mount):

    def connect(self, port="/dev/ttyUSB1"):
        print "connect > start"
        rospy.loginfo("connect > start")
        
        if port:
            self.port = port

        self.ser = serial.Serial()
        self.ser.baudrate = 9600
        self.ser.port = self.port
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = 0 ## non blocking
        self.ser.open()

        rospy.loginfo("connect > connected %s" %str(self.ser))
        self.ser.write("z")
        data = self._GetData(string = 'z')
        rospy.loginfo("actual position %s" %str(data))

        

    def _GetData(self, string, max_time = 10):
        rospy.loginfo("GetDataRequest %s" %str(string))
        self.ser.write(string)
        data = ''
        while not "#" in data:
            data = self.ser.read(999)
            if len(data) > 0:
                return data
            time.sleep(0.1)
        return data

    #def getPosition(self, param = None):
    def getPosition(self, arguments = None):
        print "get pos"
        return self._GetData('z')


        

if __name__ == '__main__':
    SynScan()