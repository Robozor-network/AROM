#!/usr/bin/env python

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
import pylirc



btn_data = []

def callback_btn(recive):
    global btn_data
    btn_data.append(recive.data)
    print recive, btn_data

class lcdInfo16x2(object):
    def __init__(self):
        rospy.Subscriber("/arom/UI/buttons", String, callback_btn)
        self.pymlab = rospy.ServiceProxy('pymlab_drive', PymlabDrive)
        rospy.init_node('lcd_info_1620')
        print "zinicializovano"

        self.pymlab(device="StatusLCD",   method="reset")
        self.pymlab(device="StatusLCD",   method="init")
        self.pymlab(device="StatusLCD",   method="clear")
        self.pymlab(device="StatusLCD",   method="home")
        '''
        time.sleep(1)
        self.pymlab(device="StatusLCD",   method="home")
        time.sleep(1)
        self.pymlab(device="StatusLCD",   method="puts", parameters=str(("Welme")))
        time.sleep(1)
        self.pymlab(device="StatusLCD",   method="set_row2")
        time.sleep(1)
        self.pymlab(device="StatusLCD",   method="puts", parameters=str(("A  AB.cz")))
        time.sleep(1)
        self.pymlab(device="StatusLCD",   method="light", parameters=str(1))
        '''
        time.sleep(1)

        '''
        lcd.clear()
        lcd.home()
        lcd.puts( "Welcome" )
        lcd.set_row2()
        lcd.puts( "AROM    MLAB.cz" % (humidity))
        lcd.light(1)
        '''


        rate = rospy.Rate(10)
        ra = 0
        dec = 90
        while not rospy.is_shutdown():
            try:
                if len(btn_data) > 0:
                    print btn_data[0], len(btn_data)
                    lastBtn = btn_data[0]
                    btn_data.pop(0)

                    print lastBtn
                    if lastBtn == 'KEY_VOLUMEUP':
                        self.pymlab(device="StatusLCD",   method="putsFull", parameters=str(dict(lineA="bla", lineB="test")))

                    elif lastBtn == 'KEY_VOLUMEDOWN':
                        self.pymlab(device="StatusLCD",   method="lcd_data", parameters=str({'i':"a"}))
                        pass

                    elif lastBtn == 'KEY_F2': 
                        self.pymlab(device="StatusLCD",   method="home")
                        self.pymlab(device="StatusLCD",   method="puts", parameters=str(dict(a="bla")))


                    elif lastBtn == 'KEY_F': 
                        self.pymlab(device="StatusLCD",   method="lightToggle")



                    elif lastBtn == 'KEY_F3':
                        pass

            except Exception, e:
                raise e


if __name__ == '__main__':
    m = lcdInfo16x2()
