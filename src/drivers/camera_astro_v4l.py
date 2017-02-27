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
from sensor_msgs.msg import Image
from arom.srv import *
from arom.msg import *
from astropy.time import Time
from __init__ import AromNode

import numpy as np
from PIL import Image
import StringIO


import getopt
import Image
import unicap
import subprocess

import camera_astro

#from camera_astro import AstroCam

   
class ImaginSource(camera_astro.AstroCam):
    node_name = "ImaginSource"
    node_type = "AstroCam"

    def __init__(self):


        self.device_id = False
        self.filetype = 'jpg'

        self.connect()
        camera_astro.AstroCam.__init__(self)

    def run(self):
        try:
            self.capture()
        except Exception, e:
            rospy.logerr(e)

    def connect(self):
        print "-----start-----------------"
        #print unicap.enumerate_devices()
        if not self.device_id:
            self.device_id = unicap.enumerate_devices()[0]
            print self.device_id

        self.device = unicap.Device( self.device_id )


        if self.filetype == 'jpg':
            self.filetype = 'jpeg'

        fmts = self.device.enumerate_formats()

        print fmts
        fmt = fmts[0]
        fmt['size'] = fmt['max_size']
        self.device.set_format( fmt )

        self.device.start_capture()


        props = self.device.enumerate_properties()
        for prop in props:
            print prop['identifier'], ">>", prop

        self.setGain(10)
        self.setFramerate(5)
        self.setExposure(1.0)


        props = self.device.enumerate_properties()
        for prop in props:
            print prop['identifier'], ">>", prop


        '''
        props = self.device.enumerate_properties()
        print "----", props
        for prop in props:
            print prop['identifier'], ">>", prop

        try:
            prop = self.device.get_property( 'Frame Rate' )
            prop['value'] = 5.0
            self.device.set_property( prop )
        except Exception, e:
            print e

        try:
            prop = self.device.get_property( 'shutter' )
            prop['value'] = 25
            self.device.set_property( prop )
        except Exception, e:
            print e


        try:
            prop = self.device.get_property( 'Gain' )
            prop['value'] = 50.0
            self.device.set_property( prop )
        except Exception, e:
            print e
        '''

    def setFramerate(self, frameRate):
        print "======================"
        try:
            #cmd = ['v4l2-ctl', '-d', 0, '-c', 'gain='+str(frameRate)]
            #r = subprocess.Popen(cmd)

            #prop = self.device.get_property( 'frame rate' )
            #prop['value'] = float(frameRate)
            #print "setFramerate", frameRate, prop
            #self.device.set_property(prop)
            pass
        except Exception, e:
            print e

    def setGain(self, Gain):
        print "======================"
        try:
            cmd = ['v4l2-ctl', '-d', str(0), '-c', 'gain='+str(Gain)]
            r = subprocess.Popen(cmd)
            #prop = self.device.get_property( 'Gain' )
            #prop['value'] = float(Gain)
            #print "SetGain", Gain, prop
            #self.device.set_property(prop)
        except Exception, e:
            print e


    def setExposure(self, exposure):
        print "======================"
        try:
            cmd = ['v4l2-ctl', '-d', str(0), '-c', 'exposure_absolute='+str(exposure)]
            r = subprocess.Popen(cmd)
            #prop = self.device.get_property( 'shutter' )
            #print prop
            #prop['value'] = float(exposure)
            #print "setExposure", exposure, prop
            #self.device.set_property(prop)
        except Exception, e:
            print e


    def capture(self, name = None, imgnum = 5, maximgnum = 10):
        if not name:
            name = "/home/odroid/capture-%s.png" %time.strftime("%m-%d-%Y_%H-%M-%s")
        try:
            #self.device.start_capture()
            print "start capture =============================="
            start = time.time()
            print self.device.enumerate_properties()
            buf = self.device.wait_buffer(40)
            rgbbuf = buf.convert('RGB3')
            #time.sleep(0.25)
            #self.device.stop_capture()
            print buf

            img = Image.frombytes( 'RGB', rgbbuf.format['size'], rgbbuf.tostring() )
            print img
            img.save( name, self.filetype )
            img.save( "/home/odroid/last.jpg", self.filetype )

            self.pub_image.publish(name)
            #self.pub_image.publish("/home/odroid/last.jpg")

            print "end of capture ===================", time.time()-start
            
        except Exception, e:
            self.device.stop_capture()
            print e

    def exit(self):
        self.device.stop_capture()
        print "ukoncuji program a odpojuji se od kameryodroid"

    def setStream(self, value):
        self.stream = bool(value)

    def streamLoop(self):
        self.capture()
        #buf = self.device.wait_buffer(40)
        #rgbbuf = buf.convert('RGB3')
        #img = Image.frombytes( 'RGB', rgbbuf.format['size'], rgbbuf.tostring() )
        #img.save("/home/odroid/last.jpg", self.filetype )
        #time.sleep(0.25)
        #self.pub_image.publish("/home/odroid/last.jpg")
        print "loop"

if __name__ == '__main__':
    ImaginSource()
    #weatherStation = AWS01B()