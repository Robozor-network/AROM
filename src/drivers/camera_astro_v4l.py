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
from astropy.time import Time

import gphoto2 as gp

import getopt
import Image
import unicap

from baseClass.camera_astro import AstroCam

   
class ImaginSource(AstroCam):
    def __init__(self):
        print "init AstroCamCanon"
        rospy.init_node('camera_astro_imaginsource')
        #super(AstroCam, self).__init__()


        self.device_id = False
        self.filetype = 'jpg'
        print "ImaginSource driver loaded"

        self.connect()

        AstroCam.__init__(self)

    def run(self):
        try:
            print "bla"
            self.capture()
        except Exception, e:
            rospy.logerr(e)

    def connect(self):
        print "-s-----------------"
        print "connect"
        print unicap.enumerate_devices()
        if not self.device_id:
            self.device_id = unicap.enumerate_devices()[0]['identifier']

        self.device = unicap.Device( self.device_id )

        print self.device.enumerate_formats()

        if self.filetype == 'jpg':
            self.filetype = 'jpeg'

        props = self.device.enumerate_properties()
        print "----", props
        for prop in props:
            print prop['identifier'], ">>", prop

        try:
            prop = self.device.get_property( 'frame rate' )
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
            prop['value'] = 62.0
            self.device.set_property( prop )
        except Exception, e:
            print e
        

        self.device.start_capture()

    def capture(self, name = None, imgnum = 5, maximgnum = 10):
        if not name:
            name = "/home/odroid/cap/capture-%s.jpg" %time.strftime("%m-%d-%Y_%H-%M-%s")
        try:
            buf = self.device.wait_buffer(60)
            print buf
        
            rgbbuf = buf.convert('RGB3')

            img = Image.fromstring( 'RGB', rgbbuf.format['size'], rgbbuf.tostring() )
            img.save( name, self.filetype )
            
        except Exception, e:
            print e



if __name__ == '__main__':
    ImaginSource()
    #weatherStation = AWS01B()