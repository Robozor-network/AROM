#!/usr/bin/env python

import math
import time
import datetime
import rospy
import std_msgs
import actionlib
import json
from std_msgs.msg import String
from std_msgs.msg import Float32
from arom.srv import *
from arom.msg import *
import numpy as np



from pydirectmount.drive import drive

from astropy import units as u
from astropy.time import Time
from astropy.coordinates import SkyCoord  # High-level coordinates
from astropy.coordinates import ICRS, Galactic, FK4, FK5, AltAz  # Low-level frames
from astropy.coordinates import Angle, Latitude, Longitude  # Angles
from astropy.coordinates import EarthLocation
from astropy.coordinates import get_sun

btn_data = []

def callback(recive):
    #for i, type in enumerate(recive.type):
    #    self.data[type] = recive.value[i]
    print recive

def callback_btn(recive):
    global btn_data
    btn_data.append(recive.data)
    print recive, btn_data

class mount(object):
    def __init__(self, parent = None, arg = None, name = "mount", port="", connect = True, var = {}):
        self.arg = arg
        self.Autoconnect = connect
        self.port = port
        self.parent = parent
        self.name = name
        self.sname = self.name
        self.variables = var
        self.rate = 1

        self.mount = drive(profile = 'HEQ5', mode = "eq", connectMethod = 'pymlab',
            obs_lat = 48.986976, obs_lon = 14.467532, obs_alt = 300, port = '/dev/ttyUSB0')
        
        self.mount.run()
        self.mount.UnPark()

        #self.mount.Slew(SkyCoord(alt = 45, az = 10, obstime = Time.now(), frame = 'altaz', unit="deg", location = self.mount.getObs()).icrs)

        rospy.Subscriber("/mount/controll", String, callback)
        rospy.Subscriber("/arom/UI/buttons", String, callback_btn)
        self.pub_status = rospy.Publisher('/mount/status', String, queue_size=10)

        rospy.init_node('AROM_mount')
        print "zinicializovano"

        rate = rospy.Rate(self.rate)
        ra = 0
        dec = 90
        while not rospy.is_shutdown():
            try:
                if len(btn_data) > 0:
                    print btn_data[0], len(btn_data)
                    lastBtn = btn_data[0]
                    btn_data.pop(0)

                    if "altaz" in lastBtn:
                        split = lastBtn.split(" ")
                        self.mount.Slew(SkyCoord(alt = float(split[1]), az = float(split[2]), obstime = Time.now(), frame = 'altaz', unit="deg", location = self.mount.getObs()).icrs)
                        

                    elif lastBtn == 'KEY_OK':
                        self.mount.Slew(SkyCoord(alt = 1, az = 180+45+90, obstime = Time.now(), frame = 'altaz', unit="deg", location = self.mount.getObs()).icrs)
                        

                        #now = Time.now()                                  
                        #altazframe = AltAz(obstime=now, location=self.mount.getObs())                                               
                        #sunaltaz = get_sun(now).transform_to(altazframe)  
                        #print sunaltaz, altazframe, now
                        #self.mount.Slew(SkyCoord(get_sun(Time.now()), location = self.mount.getObs()).icrs)

                    if lastBtn == 'KEY_UP':
                        dec += 10
                        self.mount.Slew(SkyCoord(ra = ra, dec=dec, unit="deg"))
                        #self.mount.Slew(SkyCoord(alt = 45, az = 181+90, obstime = Time.now(), frame = 'altaz', unit="deg", location = self.mount.getObs()).icrs)
                        # mode: -90 - 0

                    if lastBtn == 'KEY_DOWN':
                        dec -= 10
                        self.mount.Slew(SkyCoord(ra = ra, dec=dec, unit="deg"))
                        #self.mount.Slew(SkyCoord(alt = 45, az = 181+45, obstime = Time.now(), frame = 'altaz', unit="deg", location = self.mount.getObs()).icrs)
                        # mode: -180 - -90

                    if lastBtn == 'KEY_LEFT':
                        ra += 10
                        self.mount.Slew(SkyCoord(ra = ra, dec=dec, unit="deg"))
                        #self.mount.Slew(SkyCoord(alt = 45, az = 181, obstime = Time.now(), frame = 'altaz', unit="deg", location = self.mount.getObs()).icrs)
                        #mode: -90 - 0

                    if lastBtn == 'KEY_RIGHT':
                        ra -= 10
                        self.mount.Slew(SkyCoord(ra = ra, dec=dec, unit="deg"))
                        #self.mount.Slew(SkyCoord(alt = 1, az = 181+90, obstime = Time.now(), frame = 'altaz', unit="deg", location = self.mount.getObs()).icrs)
                        #mode: -180 - -90

                    if lastBtn == 'KEY_MENU':
                        self.mount.Slew(SkyCoord(alt = 1, az = 181+45, obstime = Time.now(), frame = 'altaz', unit="deg", location = self.mount.getObs()).icrs)

                    if lastBtn == 'KEY_TAB':
                        self.mount.Slew(SkyCoord(alt = 1, az = 181, obstime = Time.now(), frame = 'altaz', unit="deg", location = self.mount.getObs()).icrs)

                    elif lastBtn == 'KEY_F3':
                        self.mount.GoPark()

                    elif lastBtn == 'KEY_PLAY':
                        self.mount.UnPark()


            except Exception, e:
                rospy.logerr(e)
            rate.sleep()


        self.connection.close()



if __name__ == '__main__':
    m = mount()