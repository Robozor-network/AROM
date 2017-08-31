#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import rospy
#from std_msgs.msg import String
import arom
import std_msgs
import sensor_msgs
#from sensor_msgs.msg import Image
from __init__ import AromNode
from cv_bridge import CvBridge, CvBridgeError

ActionData = []


def callback_btn(recive):
    global ActionData
    ActionData.append(recive.data)
    #print recive, ActionData


class AstroCam(AromNode):
    def __init__(self):

        print "AstroCam Init"
        rospy.Subscriber("/arom/camera/controll", std_msgs.msg.String, callback_btn)
        rospy.Subscriber("/arom/UI/buttons", std_msgs.msg.String, callback_btn)
        #self.pub_image = rospy.Publisher("/arom/UI/image", sensor_msgs.msg.Image)
        rospy.Subscriber("/arom/camera/config", arom.msg.CameraParam, self.setConfig)
        self.pub_image = rospy.Publisher("/arom/camera/image", sensor_msgs.msg.Image, queue_size=0, latch=False)

        AromNode.__init__(self)
        self.set_feature('cam_controll',{'publish': '/camera/controll'})
        self.set_feature('cam_image',{'subscrib': '/camera/image'})

        self.stream = False
        self.capturing = False
        self.bridge = CvBridge()
        self.getSetting()
        i = 5


        while not rospy.is_shutdown():
            try:
                if len(ActionData) > 0:
                    lastAct = ActionData[0].split(' ')
                    print ">>>", lastAct
                    ActionData.pop(0)

                    if lastAct[0] == 'capture' or lastAct[0] == 'KEY_FN_F5':
                        print "capture"
                        self.capture()

                    elif lastAct[0] == 'startCapture':
                        print "startCapture"
                        self.capturing = True

                    elif lastAct[0] == 'stopCapture':
                        print "stopCapture"
                        self.capturing = False

                    elif lastAct[0] == 'setExpo' or  lastAct[0] == 'setExposure':
                        self.setExposure(float(lastAct[1]))


                    elif lastAct[0] == 'streamStart':
                        self.setStream(True)

                    elif lastAct[0] == 'streamStop':
                        self.setStream(False)

                    elif lastAct[0] == 'setGain':
                        self.setGain(float(lastAct[1]))

                if self.capturing:
                    print "copturing"
                    self.capture()

                elif self.stream:
                    self.streamLoop()

                else:
                    time.sleep(0.2)
                    if i > 100:
                        i = 0
                        self.getSetting()
                    i += 1

            except Exception, e:
                self.exit()
                print e

        def getCaptureName(self, extension = 'jpg'):
            return str("capture"+extension)

        def setStream(self, value):
            raise NotImplementedError

        def exit(self):
            print NotImplementedError
        
        def setFocuser(self, name = None, type = None):
            raise NotImplementedError
        
        def setConfig(self, data = None):
            raise NotImplementedError

        def setCamera(self, id = None):
            raise NotImplementedError

        def getCameraInfo(self):
            raise NotImplementedError

        def getCameralist(self):
            raise NotImplementedError

        def exit(self):
            raise NotImplementedError

        def capture(self, n=1, camera = None, exposition = None, iso = None, focus = None, Zoom = None):
            raise NotImplementedError

        def getSetting(self, name = None):
            raise NotImplementedError

