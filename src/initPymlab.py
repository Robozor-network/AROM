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
            "port": 1,
        })
    bus = str([
                {
                    "name":           "AWS_temp",
                    "type":           "lts01",
                },
                {
                    "name":           "AWS_humi",
                    "type":           "sht25",
                },{
                    "name":           "StatusLCD",
                    "type":           "i2clcd",
                },

            ])


    msg_pymlab = rospy.Publisher('pymlab_server', PymlabServerStatusM, queue_size=10)
    rospy.init_node('pymlab_client', anonymous=True)

    pymlab = rospy.ServiceProxy('pymlab_init', PymlabInit)
    print pymlab(i2c=i2c, bus=bus)
    
    msg_pymlab.publish(name = "", data="{'rate': 0.01, 'start': True, 'AutoInputs': {}}")

