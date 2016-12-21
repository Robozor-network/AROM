#!/usr/bin/env python

import rospy
from std_msgs.msg import String

ActionData = []


def callback_btn(recive):
    global ActionData
    ActionData.append(recive.data)
    print recive, ActionData


class AstroCam(object):
    def __init__(self):
        print "AstroCam Initialization"

        rospy.Subscriber("/camera/controll", String, callback_btn)
        rospy.Subscriber("/arom/UI/buttons", String, callback_btn)

        #rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                if len(ActionData) > 0:
                    print ActionData[0], len(ActionData)
                    lastAct = ActionData[0]
                    ActionData.pop(0)

                    if lastAct == 'capture' or lastAct == 'KEY_F':
                        print "capture"
                        self.capture()

            except Exception, e:
                raise e
        
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
