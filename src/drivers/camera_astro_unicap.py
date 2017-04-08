#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import getopt

import time, datetime
import rospy
import std_msgs
import sensor_msgs
import arom
from __init__ import AromNode

import numpy as np
from PIL import Image
from cv_bridge import CvBridge, CvBridgeError

import subprocess

import getopt
import Image
import unicap


import camera_astro

#from camera_astro import AstroCam

   
class ImagingSource(camera_astro.AstroCam):
    node_name = "ImagingSource"
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


    def capture(self, name = None, imgnum = 5, maximgnum = 10, save = True, publish = False):
        if not name:
            name = "/home/odroid/capture-%s.png" %time.strftime("%m-%d-%Y_%H-%M-%s")
        try:
            print "start capture =============================="
            start = time.time()
            #print self.device.enumerate_properties()
            buf = self.device.wait_buffer(40)
            print "capture 1", buf.convert
            rgbbuf = buf.format['fourcc']
            rgbbuf2 = buf.format['size']

            #rgbbuf = buf.convert('RGB3')
            print rgbbuf, rgbbuf2
            print buf

            try:
                print buf.convert('RGB3')
            except Exception as e:
                print e

            #img = Image.frombytes( 'RGB', rgbbuf.format['size'], rgbbuf.tostring() )
            #print img

            #if save:
            #    img.save(name, self.filetype)
            #if publish:
            #    self.pub_image.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

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
        self.capture(save = False, publish = True, name = "loop")
        print "loop"

if __name__ == '__main__':
    ImagingSource()
