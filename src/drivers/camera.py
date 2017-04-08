#!/usr/bin/env python
# -*- coding: utf-8 -*-

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


class Camera(object):
    def __init__(self, arg = None, connect = True, var = {}):
        self.arg = arg
        print arg
        self.Autoconnect = connect
        self.port = arg['port']
        self.parent = arg['parent']
        self.civil_name = arg['civil_name']
        self.name = arg['name']
        self.variables = var
        self.properties = {}

        ##
        ##  Inicializace vlastniho ovladace
        ##

        self.init()

        s_RegisterDriver = rospy.Service('driver/camera/%s' %self.name, arom.srv.DriverControl, self.reset)

        ##
        ##  Ceka to na spusteni AROMbrain nodu
        ##

        rospy.init_node('AROM_camera')
        rospy.loginfo("%s: wait_for_service: 'arom/RegisterDriver'" % self.name)
        rospy.wait_for_service('arom/RegisterDriver')
        rospy.loginfo("%s: >> brain found" % self.name)

        ##
        ##  Registrace zarizeni
        ##  >Arom returns 1 - OK, 0 - False
        ##

        RegisterDriver = rospy.ServiceProxy('arom/RegisterDriver', arom.srv.RegisterDriver)
        registred = RegisterDriver(name = self.name, sname= self.name, driver = self.name, device = 'camera', status = 1)
        rospy.loginfo("%s: >> register %s driver: %s" %(self.name, self.name, registred))


        ##
        ##  Ovladac se pripoji k montazi
        ##

        if self.Autoconnect:
            self.connect()

        ##
        ##  Ovladac pujde ukoncit
        ##

        rare = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            self.run()
            rare.sleep()

    def run(self):
        pass

    def reset(self, val=None):
        pass

############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################

######################################################################################
######################################################################################
##                                                                                  ##
##                  Driver for --AWS01A-- MLAB weather station                      ##
##                 ============================================                     ##
##                                                                                  ##
##                                                                                  ##
######################################################################################
        
class gphoto(Camera):
    def init(self): 
        pass

    def run(self):
        try:
            print "bla"
        except Exception, e:
            rospy.logerr(e)

    def connect(self):
        self.context = gp.gp_context_new()
        print self.context
        self.camera = gp.check_result(gp.gp_camera_new())
        print self.camera
        gp.check_result(gp.gp_camera_init(self.camera, self.context))
        text = gp.check_result(gp.gp_camera_get_summary(self.camera, self.context))
        print('Summary')
        print('=======')
        print(text.text)
        
        print('Abilities')
        print('=========')
        abilities = gp.check_result(gp.gp_camera_get_abilities(self.camera))
        print('model:', abilities.model)
        print('status:', abilities.status)
        print('port:', abilities.port)
        print('speed:', abilities.speed)
        print('operations:', abilities.operations)
        print('file_operations:', abilities.file_operations)
        print('folder_operations:', abilities.folder_operations)
        print('usb_vendor:', abilities.usb_vendor)
        print('usb_product:', abilities.usb_product)
        print('usb_class:', abilities.usb_class)
        print('usb_subclass:', abilities.usb_subclass)
        print('usb_protocol:', abilities.usb_protocol)
        print('library:', abilities.library)
        print('id:', abilities.id)
        print('device_type:', abilities.device_type)
        print "--------------"
        print abilities
        

        rospy.set_param('AROM_camera/%s/model' %(self.name), abilities.model)
        rospy.set_param('AROM_camera/%s/config' %(self.name),
            {'name': self.arg['name'],
             'civil_name': self.arg['civil_name'],
             'driver': self.arg['driver'],
             'model': abilities.model,
             '#main':{
                        '@capture': {'type': 'button', 'msg_name': 'capture'},
                        '@Shutter_speed': {'type': 'select', 'msg_name': 'shutter_speed', 'value': {'0': "1/4000", '1': "1/1000", '2': "1/800"}},
                        '@ISO': {'type': 'select', 'msg_name': 'gain', 'value': {'0': "AUTO", '1': "100", '2': "200"}},
                        '@ISO': {'type': 'select', 'msg_name': 'gain', 'value': {'0': "AUTO", '1': "100", '2': "200"}}
                    }
            }
        )

######################################################################################
######################################################################################
##                                                                                  ##
##                  Driver for --AWS01A-- MLAB weather station                      ##
##                 ============================================                     ##
##                                                                                  ##
##                                                                                  ##
######################################################################################
        
class ImaginSource(Camera):
    def init(self):
        self.device_id = False
        self.filetype = 'jpg'
        print "ImaginSource driver loaded"


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
            self.device_id = unicap.enumerate_devices()[1]['identifier']

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
            prop['value'] = 29.0
            self.device.set_property( prop )
        except Exception, e:
            print e


        try:
            prop = self.device.get_property( 'Gain' )
            prop['value'] = 32.0
            self.device.set_property( prop )
        except Exception, e:
            print e
        

        self.device.start_capture()

    def capture(self, name = "aaa.jpg", imgnum = 5, maximgnum = 10):
        try:
            buf = self.device.wait_buffer(60)
            print buf
        
            rgbbuf = buf.convert( 'RGB3' )

            img = Image.fromstring( 'RGB', rgbbuf.format['size'], rgbbuf.tostring() )
            img.save( name, self.filetype )
            
        except Exception, e:
            print e



if __name__ == '__main__':
    cfg = rospy.get_param("ObservatoryConfig/file")
    with open(cfg) as data_file:
        config = json.load(data_file)
    for x in config:
        if x['name'] == sys.argv[1]:
            break
    weatherStation = locals()[x['driver']](arg = x)
    #weatherStation = AWS01B()