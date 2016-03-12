#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from arom.srv import *
from arom.msg import *

def talker():
    pub = rospy.Publisher('arom/mount/', arom.msg.DriverControlm, queue_size=10)
    rospy.init_node('controller', anonymous=True)
    
    pub.publish(name = 'mount', type = 'Slew', data = "{'ra':%s, 'dec':%s}" %(str(sys.argv[1]), str(sys.argv[2])), validate = '', check = '')

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass