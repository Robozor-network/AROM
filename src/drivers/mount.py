#!/usr/bin/env python

import math
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
    def __init__(self, parent = None, name = "mount", port=None):
        self.port = port
        self.parent = parent
        self.name = name

        ##
        ##  Ceka to na spusteni AROMbrain nodu
        ##

        rospy.init_node('AROM_mount')
        rospy.loginfo("%s: wait_for_service: 'arom/RegisterDriver'" % self.name)
        rospy.wait_for_service('arom/RegisterDriver')
        rospy.loginfo("%s: >> done" % self.name)

        ##
        ## Registrace zarizeni
        ## >Arom returns 1 - OK, 0 - False
        ##

        RegisterDriver = rospy.ServiceProxy('arom/RegisterDriver', arom.srv.RegisterDriver)
        registred = RegisterDriver(name = self.name, driver = 'SynScan', status = 1)
        rospy.loginfo("%s: >> register %s" %(self.name, registred))

        ##
        ## Spusti se Action servis pro zmenu cile
        ##

        self.act = actionlib.SimpleActionServer('AROM/mount/target', arom.msg.MountTargetAction, execute_cb=self.ReciveTarget, auto_start = False)
        self.act.start()
        rospy.spin()


    def ReciveTarget(self, target = None):
        print "Target:", target
        rospy.loginfo('%s: GetNewTarget: %s' %(self.name, target))

    def reciveMSG(self, msg):
        pass

    def connect(self, port = None):
        print "connecting", port

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

    def getPosition(self):
        raise NotImplementedError()

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
        
class SynScan(mount):
    def connect(self, port=None):
        if port:
            self.port = port
        self.ser = serial.Serial(
            port=self.port,
            baudrate=9600,
            parity=serial.PARITY_NO,
            stopbits=serial.STOPBITS_ONE,
            #bytesize=serial.SEVENBITS
        )
        self.ser.open()
        

if __name__ == '__main__':
    mount()