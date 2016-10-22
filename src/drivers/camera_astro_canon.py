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
from std_msgs.msg import Float32MultiArray
from arom.srv import *
from arom.msg import *
#import numpy as np

from baseClass.camera_astro import AstroCam

import os
import gphoto2 as gp


class AstroCamCanon(AstroCam):
    def __init__(self):
        print "init AstroCamCanon"
        rospy.init_node('camera_astro_canon')
        #super(AstroCam, self).__init__()

        gp.check_result(gp.use_python_logging())
        self.context = gp.gp_context_new()
        self.camera = gp.check_result(gp.gp_camera_new())
        print "config"
        camera_config = gp.check_result(gp.gp_camera_get_config(self.camera, self.context))
        child_count = gp.check_result(gp.gp_widget_count_children(camera_config))
        
        print child_count
        for n in range(child_count):
            try:
                print "============"
                child = gp.check_result(gp.gp_widget_get_child(camera_config, n))
                #print child
                #label = gp.check_result(gp.gp_widget_get_label(child))
                #print label
                name = gp.check_result(gp.gp_widget_get_name(child))
                print name
                chtype = gp.check_result(gp.gp_widget_get_type(child))
                print chtype
                #value = gp.check_result(gp.gp_widget_get_value(child))
                #print value

                ro = gp.check_result(gp.gp_widget_get_readonly(child))
                print ro

                cdildcen = gp.check_result(gp.gp_widget_count_children(child))
                print cdildcen
                
                #print gp.check_result(gp.gp_widget_get_range(child))
                #print gp.check_result(gp.gp_widget_get_value(child))



            except Exception, e:
                print e

        AstroCam.__init__(self)

    def setFocuser(self, name = None, type = None):
        raise NotImplementedError

    def setCamera(self, id = None):
        raise NotImplementedError

    def getCameraInfo(self):
        raise NotImplementedError

    def getCameralist(self):
        gp.check_result(gp.use_python_logging())
        context = gp.Context()
        if hasattr(gp, 'gp_camera_autodetect'):
            cameras = context.camera_autodetect()
        else:
            port_info_list = gp.PortInfoList()
            port_info_list.load()
            abilities_list = gp.CameraAbilitiesList()
            abilities_list.load(context)
            cameras = abilities_list.detect(port_info_list, context)
        n = 0
        for name, value in cameras:
            rospy.loginfo(n, name, value)
            print(n, name, value)
            n += 1
        return 0

    def capture(self, n=1, camera = None, exposition = None, iso = None, focus = None, Zoom = None):
        try:
            gp.check_result(gp.use_python_logging())
            self.context = gp.gp_context_new()
            self.camera = gp.check_result(gp.gp_camera_new())
            gp.check_result(gp.gp_camera_init(self.camera, self.context))
            print'Capturing image', self.context, self.camera

            file_path = gp.check_result(gp.gp_camera_capture(self.camera, gp.GP_CAPTURE_IMAGE, self.context))
            print('Camera file path: {0}/{1}'.format(file_path.folder, file_path.name))
            target = os.path.join('/home/odroid/AROM/', 'aromcapt_taget_'+time.strftime("%Y%m%d-%H%M%S", time.gmtime())+'.cr2')
            print('Copying image to', target)
            camera_file = gp.check_result(gp.gp_camera_file_get(self.camera, file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL, self.context))
            gp.check_result(gp.gp_file_save(camera_file, target))
            #subprocess.call(['xdg-open', target])
            gp.check_result(gp.gp_camera_exit(self.camera, self.context))
            return True 
        except Exception, e:
            rospy.logerr(e)
            return False

    
if __name__ == '__main__':
    AstroCamCanon()