#!/usr/bin/env python

import sys
import pymlab
import rospy
import time
from pymlab import config
import sensor_server
from std_msgs.msg import String
from sensor_server.srv import *
from sensor_server.msg import *



if __name__ == "__main__":
    i2c = str({
            "device": "hid",
            "port": 1,
        })
    bus = str([
                {
                    "name":           "AWS_humi",
                    "type":           "sht31",
                },{
                    "name":           "AWS_wind_s",
                    "type":           "rps01",
                },{
                    "name":           "StatusLCD",
                    "type":           "i2clcd",
                },{
                    "name":           "AWS_temp_in",
                    "type":           "lts01",
                }#,{
                #    "name":           "io",
                #    "type":           "i2cio",
                #}

            ])

    i2c2 = str({
            "device": "smbus",
            "port": 1,
        })
    bus2 = str([
                {
                    "name":           "sht25",
                    "type":           "sht25"
                }

            ])


    msg_pymlab = rospy.Publisher('pymlab_server', PymlabServerStatusM, queue_size=10)
    rospy.init_node('pymlab_client', anonymous=True)

    pymlab = rospy.ServiceProxy('pymlab_init', PymlabInit)
    print pymlab(i2c=i2c, bus=bus)
    #print pymlab(i2c=i2c2, bus=bus2)
    
    msg_pymlab.publish(name = "", data="{'rate': 0.01, 'start': True, 'AutoInputs': {}}")

