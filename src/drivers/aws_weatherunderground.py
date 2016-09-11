#!/usr/bin/env python

import math
import time
import rospy
from arom.srv import *
from arom.msg import *
import httplib2


data = {}

def callback(recive):
    global data
    for i, type in enumerate(recive.type):
        data[type] = recive.value[i]
    print data


def weatherUploader():
    name = 'aws_weatherudnerground'
    rospy.init_node(name, anonymous=True)
    rospy.Subscriber("/aws_out", msg_WeatherStation, callback)

    ID = rospy.get_param('/arom/aws/%s/ID'%(name))
    PASSWORD = rospy.get_param('/arom/aws/%s/PASSWORD'%(name))

    rate = rospy.Rate(0.01) # 10hz
    while data == {} and not rospy.is_shutdown():
        time.sleep(0.25)
    while not rospy.is_shutdown():

        req = "?ID=%s&PASSWORD=%s&dateutc=now&softwaretype=AROM-AWS" %(ID , PASSWORD)
        req += "&tempf=%f" %(data['temperatureAWS0']*9/5+32)
        req += "&dewptf=%f" %(data['dewpointAWS']*9/5+32)
        req += "&humidity=%f" %(data['humidityAWS0'])
        req += "&winddir=%f" %(data['winddirAWS'])
        req += "&windspeedmph=%f" %(data['windspdAWS'])
        req += "&baromin=%f" %(data['pressureAWS']*0.0002952998751)

        rospy.loginfo("Uploading data to weatherudnerground.com: %s" %repr(req))
        resp, content = httplib2.Http().request("http://weatherstation.wunderground.com/weatherstation/updateweatherstation.php"+req)

        rate.sleep()


if __name__ == '__main__':
    weatherUploader()