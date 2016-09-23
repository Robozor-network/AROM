#!/usr/bin/env python

import sys
import pymlab
import rospy
import time
from pymlab import config
import arom
from arom.msg import *
from arom.srv import *
from std_msgs.msg import String
#from sensor_server.srv import *
#from sensor_server.msg import *



if __name__ == "__main__":
    i2c = str({
                "device": "smbus",
                "port": 1,
    })
    bus = str([
                {
                    "name":           "AWS_humi",
                    "type":           "sht31",
                    "address":        0x44,
                #},{
                  #  "name":           "AWS_humi",
                  #  "type":           "sht31",
                  #  "address":        0x00,
                },{
                    "name":           "AWS_wind_s",
                    "type":           "rps01",
                    "address":        0x40,
                },{
                    "name":           "AWS_temp_ref",
                    "type":           "lts01",
                    "address":        0x48,
                },{
                    "name":           "AWS_wind_d",
                    "type":           "mag01",
                    "gauss":          0.88,
                    "address":        0x1E,
                },{
                    "name":           "AWS_press",
                    "type":           "altimet01",
                    "address":        0x60,
                },{
                    "name":           "AWS_light",
                    "type":           "isl03",
                    "address":        0x10,
                },{
                    "name":           "AWS_clouds",
                    "type":           "thermopile01",
                    "address":        0x5d,
                },{
                    "name":           "StatusLCD",
                    "type":           "i2clcd"
                }#,{
                #    "name":           "AWS_humi_in",
                #    "type":           "sht25"
                #}#,{
                #    "name":           "dd_temp_g0a",
                #    "type":           "sht25"
                #}
                #,{
                #    "name":           "dd_heater_g0a",
                #    "type":           "i2cpwm",
                #}#{
                #    "name":           "io",
                #    "type":           "i2cio",
                #},
                
    ])
    
    i2c2 = str({
            "device": "serial",
            "port": '/dev/ttyUSB1',
        })
    bus2 = str([
                {
                    "name":           "telescope_lts",
                    "type":           "lts01"
                },{ 
                    "name":           "telescope_spi", 
                    "type":           "i2cspi"
                }#,{
                #    "name":           "telescope_magnetometer",
                #    "type":           "mag01",
                #    "gauss":          0.88,
                #    "address":        0x1E,
                #}
                
            ])
    




    msg_pymlab = rospy.Publisher('pymlab_server', msg_pymlabInit, queue_size=10)
    rospy.init_node('pymlab_client', anonymous=True)

    pymlab = rospy.ServiceProxy('pymlab_init', PymlabInit)
    print pymlab(i2c=i2c, bus=bus)
    print pymlab(i2c=i2c2, bus=bus2)
    
    msg_pymlab.publish(name = "", data="{'start': True}")
