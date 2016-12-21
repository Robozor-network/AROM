#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time
import rospy
import std_msgs
import json
from std_msgs.msg import String
from std_msgs.msg import Float32
from arom.srv import *
from arom.msg import *
import numpy as np
from __init__ import AromNode

btn_data = []

def callback_btn(recive):
    print recive
    global btn_data
    btn_data.append(recive.data)
    #print recive, btn_data


class laserPointer(AromNode):
    node_name = "laserPointer_mount"
    node_type = "laser"
    node_pymlab = True

    def __init__(self):
        self.pymlab2 = rospy.ServiceProxy('pymlab_drive', PymlabDrive)

        rospy.Subscriber("/mount/laser", String, callback_btn)
        AromNode.__init__(self)
        self.pymlab2(device="gpio_mount", method="config_ports", parameters=str(dict(port0 = 0x00, port1 = 0x00)))
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            try:
                if len(btn_data) > 0:
                    print btn_data[0], len(btn_data)
                    lastBtn = btn_data[0]
                    btn_data.pop(0)

                    print lastBtn


                    if lastBtn == 'ON':
                        print "Laser ON"
                        self.pymlab2(device="gpio_mount", method="set_ports", parameters=str(dict(port0 = 0b11111111, port1= 0b11111111)))

                    elif lastBtn == 'OFF':
                        print "Laser OFF"
                        self.pymlab2(device="gpio_mount", method="set_ports", parameters=str(dict(port0 = 0b00000000, port1= 0b00000000)))

            except Exception, e:
                print e
            rate.sleep()



if __name__ == '__main__':
    m = laserPointer()