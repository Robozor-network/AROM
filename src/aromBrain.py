#!/usr/bin/env python

import sys
import rospy
import pymlab
from pymlab import config
import std_msgs
from std_msgs.msg import String
from std_msgs.msg import Float32
from arom.srv import *
from arom.msg import *

import json
import drivers
#from drivers import devices
#from drivers import camera
#from drivers import focuser
#from drivers import weatherStation
#from drivers import rotator
#from drivers import roof
#import drivers


class AromBrain():
    def mountSlew(self, cor = [0,0]):
        pass

    def __init__(self):
        rospy.init_node('AROM_brain')
        self.mount = {}
        self.camera = {}
        self.devices = {}

        from drivers import mount, camera, roof, weatherStation, focuser, rotator
        self.drivers = {
            'EQmod': mount.EQmod,
            'SynScan': mount.SynScan,
            'mount': drivers.mount.EQmod,
        }

        s_RegisterDriver = rospy.Service('arom/RegisterDriver', arom.srv.RegisterDriver, self.RegisterDriver)

        self.config_file = sys.argv[1].decode('utf-8')
        with open(self.config_file) as data_file:    
            self.config = json.load(data_file)
        rospy.set_param("ObservatoryConfig/file", str(self.config_file))

        '''
        rospy.wait_for_service('arom/mount/parameter')
        mount = rospy.ServiceProxy('arom/mount/parameter', arom.srv.MountParameter)
        registred = mount(name = 'getPosition', value = '')
        registred = mount(name = 'getTime', value = '')
        rospy.loginfo("%s: >> position %s" %("brain", registred))
        Actmount = rospy.ServiceProxy('arom/mount/parameter', arom.srv.MountParameter)
        '''

        '''
        client = actionlib.SimpleActionClient('fibonacci', actionlib_tutorials.msg.FibonacciAction)
        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()
        # Creates a goal to send to the action server.
        goal = actionlib_tutorials.msg.FibonacciGoal(order=20)
        # Sends the goal to the action server.
        client.send_goal(goal)
        # Waits for the server to finish performing the action.
        client.wait_for_result()
        # Prints out the result of executing the action
        return client.get_result() 
        '''

        '''
        for x in xrange(1,5):
            registred = mount(name = 'getPosition', value = '')
            time.sleep(5)
        '''

        rospy.spin()

    def RegisterDriver(self, srv):
        #rospy.loginfo("NewDevice>> type: %s, name %s (%s). With driver %s" %(srv.device, srv.name, srv.sname, srv.driver))
        dev_driver = self.drivers[srv.driver]
        eval('self.'+srv.device)[srv.sname] = {'name':srv.name, 'sname':srv.sname, 'driver':srv.driver, 'object':dev_driver}
        self.devices[srv.sname] = {'name':srv.name, 'sname':srv.sname, 'driver':srv.driver, 'object':dev_driver}
        rospy.loginfo("NewDevice>> type: %s, name %s (%s). With driver %s >> %s" %(srv.device, srv.name, srv.sname, srv.driver, str(self.devices[srv.sname])))
        return 1

    def loadDriver(self, deviceType = None, driverName = 'mount'):

        try:
            driver = None
            if deviceType:
                driver = getattr(eval('mount'), driverName)(self)
        except Exception, e:
            print "Error:", e
        finally:
            return driver

    def mount_move(self, target = [10,10]):
        print 
                    


def main():

    #rospy.Subscriber("pymlab_server", PymlabServerStatusM, ps.status)
    #s1 = rospy.Service('pymlab_init', PymlabInit, ps.init)
    #s2 = rospy.Service('pymlab_server', PymlabServerStatus, ps.status)
    #s3 = rospy.Service('pymlab_drive', PymlabDrive, ps.drive)

    #rospy.loginfo("Ready to get work.")
    #rospy.spin()

    ab = AromBrain()

if __name__ == "__main__":
    main()