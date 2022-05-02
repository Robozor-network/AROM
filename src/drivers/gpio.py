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


class gpio_mount(AromNode):
    node_name = "gpio_mount"
    node_type = "gpio"
    node_pymlab = True

    def __init__(self):
        #self.pymlab2 = rospy.ServiceProxy('pymlab_drive', PymlabDrive)

        self.config0 = 0x00
        self.config1 = 0x00
        self.port0 = 0b00000000
        self.port1 = 0b00000000
        self.devices0 = ['mount_pwr', 'laser_pwr', '12V_3', '12V_4', 'laser_status', None, None, None]
        #self.devices0 = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
        self.devices1 = [None, None, None, None, None, None, None, None]

        rospy.Subscriber("/gpio/gpio_mount", String, callback_btn)
        self.pub_status = rospy.Publisher('/gpio/gpio_status', String, queue_size=10)

        AromNode.__init__(self)
        self.set_feature('gpio_set_port',{'ports': 8, 'devices': str(self.devices0) ,'subscrib': '/gpio/gpio_mount'})
        self.set_feature('gpio_all_off',{'subscrib': '/gpio/gpio_mount', 'msg': 'off'})

        self.pymlab(device="gpio_mount", method="config_ports", parameters=str(dict(port0 = self.config0, port1 = self.config1)))
        self.pymlab(device="gpio_mount", method="set_ports", parameters=str(dict(port0 = self.port0, port1 = self.port1)))

        rate = rospy.Rate(5)
        rospy.Timer(rospy.Duration(2), self.send_status, oneshot=False)
        while not rospy.is_shutdown():
            try:

                if len(btn_data) > 0:
                    print btn_data[0], len(btn_data)
                    lastBtn = btn_data[0]
                    btn_data.pop(0)


                    print "incomming:", lastBtn

                    last_data = lastBtn.split(" ")

                    if last_data[0] in self.devices0:
                        bit = self.devices0.index(last_data[0])
                        self.port0 = abs(self.port0)
                        print ">>", bin(self.port0)[2:].zfill(8), bit

                        if last_data[1] == "1":
                            print "set", 1, bit
                            self.port0 |= (1 << bit) #nastavit bit

                        elif last_data[1] == "0":
                            print "clear", 0, bit
                            self.port0 &= ~(1 << bit) #nastavit bit

                        elif last_data[1] == 'toggle':
                            self.port0 ^= (1 << bit) #smazat bit
                        
                        elif last_data[1] == 'off':
                            self.port0 = 0x00 #smazat bit
                        
                        self.port0 = abs(self.port0)
                        print "<<", bin(self.port0)[2:].zfill(8)
                    
                    self.pymlab(device="gpio_mount", method="set_ports", parameters=str(dict(port0 = self.port0, port1= self.port1)))

            except Exception, e:
                print e
            rate.sleep()


    def send_status(self, object):
        print "posilam status"
        #print self.pymlab(device="gpio_mount", method="get_ports").value
        port0, port1 = eval(self.pymlab(device="gpio_mount", method="get_ports").value)
        self.pub_status.publish(str(bin(port0)[2:].zfill(8))+str(bin(port1)[2:].zfill(8)))
            

if __name__ == '__main__':
    m = gpio_mount()