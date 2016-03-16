#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
import time
from arom.srv import *
from arom.msg import *

import astropy.units as u
from astropy.coordinates import SkyCoord
from astropy.coordinates import EarthLocation
from astropy.time import Time

observatory = EarthLocation(lat=49*u.deg, lon=14*u.deg, height=300*u.m)

def talker():
    pub = rospy.Publisher('arom/mount/', arom.msg.DriverControlm, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    while True:
        var = raw_input("Enter: ")
        var = var.split(' ')
       # print var

        if var[0] == 'radec':
            pub.publish(name = 'mount', type = 'Slew', data = "{'ra':%s, 'dec':%s}" %(str(var[1]), str(var[2])), validate = '', check = '')

        elif var[0] == 'altaz':
            coord = SkyCoord(alt = float(var[1]) * u.deg, az = float(var[2]) * u.deg, obstime = Time.now(), frame = 'altaz', location = observatory)
            print coord.icrs.to_string('hmsdms')
            pub.publish(name = 'mount', type = 'Slew', data = "{'ra':%s, 'dec':%s}" %(str(coord.icrs.ra.degree), str(coord.icrs.dec.degree)), validate = '', check = '')
        elif var[0] == 'q':
            break
        else:
            print "no value"

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass