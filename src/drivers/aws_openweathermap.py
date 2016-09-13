#!/usr/bin/env python

import math
import time
import rospy
from arom.srv import *
from arom.msg import *
import httplib2
import urllib


data = {}

def callback(recive):
    global data
    for i, type in enumerate(recive.type):
        data[type] = recive.value[i]
    print data


def listener():

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/aws_out", msg_WeatherStation, callback)


    rate = rospy.Rate(0.1) # 10hz
    while data == {} and not rospy.is_shutdown():
        time.sleep(0.25)
    while not rospy.is_shutdown():
        print "bla", data

        post_data = {'name':"ZVPP",
                     'lat': 48.986976,
                     'long': 14.467532,
                     'alt': 350,
                     'pressure': data['pres_out'],
                     'humidity': data['humidity_sht'],
                     'temp': data['temp_sht'],
                     'dewpoint': data['dew_pt'],
                     'wind_speed': data['windspeed_out'],
                     'wind_dir': data['winddir_out'],
                     'lum': data['light_out'],
                     }

        print httplib2.Http().request("http://openweathermap.org/data/post", 'POST', urllib.urlencode(post_data))

        rate.sleep()


if __name__ == '__main__':
    listener()