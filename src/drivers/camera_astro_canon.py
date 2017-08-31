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
from std_msgs.msg import Float32MultiArray
from arom.srv import *
from arom.msg import *
#import numpy as np

import camera_astro
import rawpy
import imageio

import os
import gphoto2 as gp
import piggyphoto

class AstroCamCanon(camera_astro.AstroCam):
    node_name = "CanonCamera"
    node_type = "AstroCam"

    def __init__(self):
        print "init AstroCamCanon"

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
                name = gp.check_result(gp.gp_widget_get_name(child))
                print name
                chtype = gp.check_result(gp.gp_widget_get_type(child))
                print chtype
                ro = gp.check_result(gp.gp_widget_get_readonly(child))
                print ro
                cdildcen = gp.check_result(gp.gp_widget_count_children(child))
                print cdildcen
                

            except Exception, e:
                print e

        camera_astro.AstroCam.__init__(self)

    def setFocuser(self, name = None, type = None):
        raise NotImplementedError

    def setCamera(self, id = None):
        raise NotImplementedError

    def getCameraInfo(self):
        raise NotImplementedError

    def getCaptureName(self, extension = 'jpg'):
        return str("capture"+extension)

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

    def publishCR2(self, file):
        print "publishCR2", file
        raw = rawpy.imread(file)
        #rgb = raw.postprocess(gamma=(1,1), no_auto_bright=True, output_bps=16)
        rgb = raw.postprocess()
        imageio.imsave(file+'.tiff', rgb)
        print "save"
        bayer = raw.raw_image
        print "bayer"
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(cv2.fromarray(bayer), "bgr8"))


    def capture(self, n=1, camera = None, exposition = None, iso = None, focus = None, Zoom = None, save = True, publish = False, name = None):
        T_start = time.time()
        try:
            if not name:
                name = self.getCaptureName(extension = '')
            filename = '/home/odroid/robozor/', name + '_'+time.strftime("%Y%m%d-%H%M%S", time.gmtime())+'.cr2'
            gp.check_result(gp.use_python_logging())
            self.context = gp.gp_context_new()
            self.camera = gp.check_result(gp.gp_camera_new())
            gp.check_result(gp.gp_camera_init(self.camera, self.context))
            print'Capturing image', self.context, self.camera

            file_path = gp.check_result(gp.gp_camera_capture(self.camera, gp.GP_CAPTURE_IMAGE, self.context))
            print('Camera file path: {0}/{1}'.format(file_path.folder, file_path.name))
            target = os.path.join(filename)
            self.publishCR2(file = filename)
            print('Copying image to', target)
            camera_file = gp.check_result(gp.gp_camera_file_get(self.camera, file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL, self.context))
            gp.check_result(gp.gp_file_save(camera_file, target))
            gp.check_result(gp.gp_camera_exit(self.camera, self.context))
            return True 
        except Exception, e:
            rospy.logerr(e)
            return False
        T_duration = time.time() - T_start
        print "Doba prikazu: %s s" %(T_duration)

    def exit(self):
        print "ukoncuji program a odpojuji se od kamery"

    def setConfig(self, data):
        print "Data setConfig", data

    def getSetting(self):
        print "getSetting"
        conf_dict = {}
            
        rospy.set_param('/arom/node%s/feature/%s/config' %(str(rospy.get_name()),'cam_controll'), conf_dict)
        print "***********************"

    def setStream(self, value):
        self.stream = bool(value)

    def streamLoop(self):
        path = piggyphoto.CameraFilePath()
        cfile = piggyphoto.cameraFile()

        ans = 0
        for i in range(1 + 1000):
            ans = gp.gp_camera_capture_preview(self.camera, self.context)
            if ans == 0: break
            else: print "capture_preview(%s) retry #%d..." % (destpath, i)
        #check(ans)
        print ">>>>", ans

        #if destpath:
        #    cfile.save(destpath)
        #else:
        #    return cfile

        print "loop"

    
if __name__ == '__main__':
    AstroCamCanon()