#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Python libs
import sys, time

# numpy and scipy
import numpy as np
#from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
# We do not use cv_bridge it does not support CompressedImage in python
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE=False

class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
            CompressedImage)
        # self.bridge = CvBridge()

        rospy.init_node('AROM_camera')

        cap = cv2.VideoCapture(0)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            try:
                #data = self.mesure()
                #self.send(data)

                print "----------"
                ret = False
                ret, frame = cap.read()

                print ret

                # Our operations on the frame come here
                #frame = cv2.imread('/home/odroid/roman')
                image_np = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


                print "aa"
                #### Create CompressedIamge ####
                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.format = "jpeg"
                msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
                # Publish new image
                self.image_pub.publish(msg)
                
                #self.subscriber.unregister()

            except Exception, e:
                rospy.logerr(e)
            rate.sleep()

def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    rospy.init_node('image_feature', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)