#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys

import math
import time
import datetime
import rospy
import std_msgs
import actionlib
import json
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from arom.srv import *
from arom.msg import *
import numpy as np
import pprint 
pp = pprint.PrettyPrinter(indent=4)

#from arom_tools import *
from __init__ import AromNode

from pydirectmount.drive import drive

from astropy import units as u
from astropy.time import Time
from astropy.coordinates import SkyCoord  # High-level coordinates
from astropy.coordinates import ICRS, Galactic, FK4, FK5, AltAz  # Low-level frames
from astropy.coordinates import Angle, Latitude, Longitude  # Angles
from astropy.coordinates import EarthLocation
from astropy.coordinates import get_sun #, get_body
#from astropy.coordinates import solar_system_ephemeris
from astroquery.simbad import Simbad

btn_data = []

def callback(recive):
    #for i, type in enumerate(recive.type):
    #    self.data[type] = recive.value[i]
    print recive

def callback_btn(recive):
    global btn_data
    btn_data.append(recive.data)
    print recive, btn_data

class mount(AromNode):
    node_name = "AROM_mount"
    node_type = "mount"
    node_pymlab = True

    def __init__(self, parent = None, arg = None, name = "mount", port="", connect = True, var = {}):
        self.arg = arg
        self.Autoconnect = connect
        self.port = port
        self.parent = parent
        self.name = name
        self.sname = self.name
        self.variables = var
        self.rate = 5

        self.speeds = [0, 10, 20, 40, 80, 160]
        self.raSpeedType = 0
        self.decSpeedType = 0

        self.mount = drive(profile = 'HEQ5', mode = "eq", connectMethod = 'pymlab',
            obs_lat = 48.986976, obs_lon = 14.467532, obs_alt = 382, port = '/dev/ttyUSB0')
        
        self.mount.run()
        self.mount.UnPark()

        #self.mount.Slew(SkyCoord(alt = 45, az = 10, obstime = Time.now(), frame = 'altaz', unit="deg", location = self.mount.getObs()).icrs)

        rospy.Subscriber("/mount/controll", String, callback_btn)
        rospy.Subscriber("/mount/set/offset2", String, self.set_offset)
        rospy.Subscriber("/arom/UI/buttons", String, callback_btn)
        rospy.Subscriber("/ui/Keyboard/out", String, callback_btn)
        self.pub_status = rospy.Publisher('/mount/status', String, queue_size=10)
        self.pub_radec  = rospy.Publisher('/mount/status/coordinates/RaDec', Float32MultiArray, queue_size=10)
        self.pub_radec_mount  = rospy.Publisher('/mount/status/coordinates/RaDec_mount', Float32MultiArray, queue_size=10) # souradnice, ktere nemaji chybu offsetu
        self.pub_motor_a = rospy.Publisher("/mount/status/motor/ra", std_msgs.msg.String, queue_size=10)
        self.pub_motor_b = rospy.Publisher("/mount/status/motor/dec", std_msgs.msg.String, queue_size=10)

        #rospy.init_node('AROM_mount')
        AromNode.__init__(self)
        #self.set_feature('mount_position',{'publish': '/mount/status/coordinates/RaDec'})
        #self.set_feature('mount_offset',{'subscrib': '/mount/controll'})
        self.set_feature('mount_slew',{'subscrib': '/mount/controll', 'publish': '/mount/status/coordinates/RaDec'})
        self.set_feature('mount_tracking',{'subscrib': '/mount/controll', 'publish': '/mount/status/coordinates/RaDec'})

        self.set_feature('hbstep_status__1',  {'id': 'motor_a', 'name': 'Motor RA',  'toppic': '/mount/status/motor/ra'})
        self.set_feature('hbstep_status__2',  {'id': 'motor_b', 'name': 'Motor DEC', 'toppic': '/mount/status/motor/dec'})

        self.set_feature('mount_skymap',{})
        #self.set_feature('mount_info',{'type': 'HEQ5', 'mount_mode': 'eq', 'obs_lat': 10.2332, 'obs_lon': 10.2332, 'obs_alt': 10.2332})

        (trackSpd_ra, trackSpd_dec) = self.mount.getDefaultTrackingSpd()
        print (trackSpd_ra, trackSpd_dec)

        print "zinicializovano"

        rate = rospy.Rate(self.rate)
        ra = 0
        dec = 90
        rospy.Timer(rospy.Duration(1), self.sendPosition, oneshot=False)
        while not rospy.is_shutdown():
            try:
                if len(btn_data) > 0:
                    print btn_data[0], len(btn_data)
                    lastBtn = btn_data[0]
                    btn_data.pop(0)

                    if "name" in lastBtn:
                        split = lastBtn.split(" ")
                        self.mount.Slew(SkyCoord.from_name(split[1]))

                    #elif "solar" in lastBtn:
                    #    split = lastBtn.split(" ")
                    #    self.mount.Slew(get_body(split[1], time = Time.now(), location = self.mount.getObs()).icrs)

                    elif "sun" in lastBtn:
                        print get_sun(Time.now()).icrs

                    elif "altaz" in lastBtn:
                        split = lastBtn.split(" ")
                        self.mount.Slew(SkyCoord(alt = float(split[1]), az = float(split[2]), obstime = Time.now(), frame = 'altaz', unit="deg", location = self.mount.getObs()).icrs)
                    
                    elif "radec" in lastBtn:
                        split = lastBtn.split(" ")
                        self.mount.Slew(SkyCoord(ra = float(split[1]), dec = float(split[2]), obstime = Time.now(), unit="deg", location = self.mount.getObs()).icrs)
                        
                    elif "tle" in lastBtn:
                        split = lastBtn.split(" ")
                        self.mount.StartTrackingTLE(name = split[1])
                        #self.mount.Slew(SkyCoord(alt = float(split[1]), az = float(split[2]), obstime = Time.now(), frame = 'altaz', unit="deg", location = self.mount.getObs()).icrs)
                        
                    elif "resetMount" in lastBtn:
                        self.mount.Reset()

                    elif "spd" in lastBtn:
                        split = lastBtn.split(" ")
                        trackSpd_ra += float(split[1])
                        trackSpd_dec += float(split[2])
                        self.mount.setTrackingSpeed(ra = float(split[1]), dec = float(split[2]))
                        print split[1], split[2]
                        self.mount.tracking(True)
                        
                    elif lastBtn in ["startTracking"]:
                        self.mount.tracking(True, tracking_mode='sidereal')
                        
                    elif lastBtn in ['stopTracking', 'STOP1']:
                        trackSpd_ra, trackSpd_dec = 1.7, 0
                        self.mount.setTrackingSpeed(ra = trackSpd_ra, dec = trackSpd_dec)
                        self.mount.tracking(True)
                        
                    elif lastBtn == 'home' or lastBtn == 'KEY_STOP':
                        self.mount.GoPark()

                    elif lastBtn == 'KEY_OK':
                        self.mount.Slew(SkyCoord(ra = float(split[1]), dec = float(split[2]), frame = 'icrs', unit="deg"))

                    elif lastBtn in ['KEY_UP', 'DEC_P1']:
                        trackSpd_dec += 10
                        self.mount.setTrackingSpeed(ra = float(trackSpd_ra), dec = float(trackSpd_dec))
                        self.mount.tracking(True)
                        #dec += 10
                        #self.mount.Slew(SkyCoord(ra = ra, dec=dec, unit="deg"))

                    elif lastBtn in ['KEY_DOWN', 'DEC_M1']:
                        trackSpd_dec -= 10
                        self.mount.setTrackingSpeed(ra = float(trackSpd_ra), dec = float(trackSpd_dec))
                        self.mount.tracking(True)
                        #dec -= 10
                        #self.mount.Slew(SkyCoord(ra = ra, dec=dec, unit="deg"))

                    elif lastBtn in ['KEY_LEFT', 'RA_P1']:
                        trackSpd_ra += 10
                        self.mount.setTrackingSpeed(ra = float(trackSpd_ra), dec = float(trackSpd_dec))
                        self.mount.tracking(True)
                        #ra += 10
                        #self.mount.Slew(SkyCoord(ra = ra, dec=dec, unit="deg"))

                    elif lastBtn in ['KEY_RIGHT', 'RA_M1']:
                        trackSpd_ra -= 10
                        self.mount.setTrackingSpeed(ra = float(trackSpd_ra), dec = float(trackSpd_dec))
                        self.mount.tracking(True)
                        #ra -= 10
                        #self.mount.Slew(SkyCoord(ra = ra, dec=dec, unit="deg"))

                    elif lastBtn == 'KEY_MENU':
                        pass
                        #self.mount.Slew(SkyCoord(alt = 1, az = 181+45, obstime = Time.now(), frame = 'altaz', unit="deg", location = self.mount.getObs()).icrs)

                    elif lastBtn == 'KEY_TAB':
                        pass
                        #self.mount.Slew(SkyCoord(alt = 1, az = 181, obstime = Time.now(), frame = 'altaz', unit="deg", location = self.mount.getObs()).icrs)

                    elif lastBtn == 'KEY_PLAY' or lastBtn == "unpark":
                        self.mount.UnPark()
                else:
                    #print "Something else ....."
                    #(ra, dec) = self.mount.getCoordinates('RaDec')
                    #print ra, dec

                    #try:
                    #    mat = Float32MultiArray(data=[ra, dec])
                    #    self.pub_radec.publish(mat)

                    #except Exception, e:
                    #    print e
                    #mat = Float32MultiArray(data=[ra, dec])
                        #mat.layout.dim.append(MultiArrayDimension())
                        #mat.layout.dim[0].label = "RaDec"
                        #mat.layout.dim[0].size = 2
                        #mat.data.
                    #print mat
                    #self.pub_radec.publish(mat)
                    pass


            except Exception, e:
                rospy.logerr(e)
            rate.sleep()


        self.connection.close()

    def set_offset(self, msg):
        array =  msg.data.split(";")
        print array
        self.mount.setOffset(ra=array[0], dec=array[1])

    def sendPosition(self, object):
        try:   
            coord = self.mount.getCoordinates(sky = True)
            coord_mount = self.mount.getCoordinates(sky = False)
            motor_a, motor_b = self.mount.getStepperStatus()

            mat = Float32MultiArray(data=[coord.ra.degree, coord.dec.degree])
            self.pub_radec.publish(mat)
            mat = Float32MultiArray(data=[coord_mount.ra.degree, coord_mount.dec.degree])
            self.pub_radec_mount.publish(mat)

            if motor_a:
                self.pub_motor_a.publish(json.dumps(motor_a, ensure_ascii=False))
            if motor_b:
                self.pub_motor_b.publish(json.dumps(motor_b, ensure_ascii=False))
        except Exception, e:
            print "send", e
            



if __name__ == '__main__':
    m = mount()
