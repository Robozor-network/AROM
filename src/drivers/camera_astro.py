#!/usr/bin/env python

import rospy
#from std_msgs.msg import String
import std_msgs
import sensor_msgs
from __init__ import AromNode

ActionData = []


def callback_btn(recive):
    global ActionData
    ActionData.append(recive.data)
    #print recive, ActionData


class AstroCam(AromNode):
    def __init__(self):

        rospy.Subscriber("/arom/camera/controll", std_msgs.msg.String, callback_btn)
        rospy.Subscriber("/arom/UI/buttons", std_msgs.msg.String, callback_btn)
        #self.pub_image = rospy.Publisher("/arom/UI/image", sensor_msgs.msg.Image)
        self.pub_image = rospy.Publisher("/arom/camera/image", std_msgs.msg.String, queue_size=3, latch=False)

        AromNode.__init__(self)
        self.set_feature('cam_controll',{'publish': '/camera/controll'})
        self.set_feature('cam_image',{'subscrib': '/camera/image'})

        self.stream = False


        while not rospy.is_shutdown():
            try:
                if len(ActionData) > 0:
                    lastAct = ActionData[0].split(' ')
                    print ">>>", lastAct
                    ActionData.pop(0)

                    if lastAct[0] == 'capture' or lastAct[0] == 'KEY_FN_F5':
                        print "capture"
                        self.capture()

                    elif lastAct[0] == 'setExpo' or  lastAct[0] == 'setExposure':
                        self.setExposure(float(lastAct[1]))


                    elif lastAct[0] == 'streamStart':
                        self.setStream(True)

                    elif lastAct[0] == 'streamStop':
                        self.setStream(False)

                    elif lastAct[0] == 'setGain':
                        self.setGain(float(lastAct[1]))

                if self.stream:
                    self.streamLoop()

            except Exception, e:
                self.exit()
                print e

        def setStream(self, value):
            raise NotImplementedError

        def exit(self):
            print NotImplementedError
        
        def setFocuser(self, name = None, type = None):
            raise NotImplementedError

        def setCamera(self, id = None):
            raise NotImplementedError

        def getCameraInfo(self):
            raise NotImplementedError

        def getCameralist(self):
            raise NotImplementedError

        def capture(self, n=1, camera = None, exposition = None, iso = None, focus = None, Zoom = None):
            raise NotImplementedError
