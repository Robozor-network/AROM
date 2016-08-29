#!/usr/bin/env python

import rospy
import pymlab
from pymlab import config
import sys
import arom
from std_msgs.msg import String
from std_msgs.msg import Float32
import std_msgs
import time
from arom.srv import *
from arom.msg import *


def server(req):
    print req
    print "Returning [%s + %s]"%(req.name, req.data)
    return GetSensValResponse( 10 )

class pymlab_server():
    def __init__(self):
        self.err_count = 0
        self.pymlab_read = False # slouzi k ochrane pymlabu pred pokusem o dve cteni zaroven ...
        self.devices = {}
        self.i2c = []
        self.bus = []


    def init(self, cfg=None):
        try:
            self.status = False
            self.init = cfg

            self.cfg_i2c = eval(cfg.i2c)
            self.i2c.append(self.cfg_i2c)
            self.cfg_bus = eval(cfg.bus)
            self.bus.append(self.cfg_bus)

            rospy.loginfo("konfiguracni data: %s" %str(cfg))
            self.init_bus(i2c = self.cfg_i2c, bus = self.cfg_bus)

            return True
        except Exception, e:
            rospy.logerr("#03:"+repr(e))
            return False

    def init_bus(self, i2c, bus):
        self.pymlab_config = config.Config(i2c = i2c, bus = bus)
        self.pymlab_config.initialize()
        for x in bus:
            try:
                self.devices[x['name']] = self.pymlab_config.get_device(x['name'])
            except Exception, e:
                rospy.logerr("#02:"+repr(e))
            print x['name'], 
            self.devices[x['name']] = self.pymlab_config.get_device(x['name'])
            print self.devices[x['name']]

    def getvalue(self, cfg=None):
        val = int(float(self.lts_sen.get_temp()))
        return GetSensValResponse(val)

    def status(self, cfg=None):
        self.rate = 1
        rospy.loginfo("prijaty cfg: %s" %(str(cfg)))
        try:                            # pokud je servis s GetSensVal, tak se pouzije toto, 
            ecfg = eval(cfg.data)
        except Exception, e:            # toto je pro zpravu 'pymlab_server'
            ecfg = eval(cfg)
        print ecfg
        if 'rate' in ecfg:
            self.rate = ecfg['rate']
            #print "Vlastni frekvence", self.rate
        rospy.set_param("rate", float(self.rate))
        if 'AutoInputs' in ecfg:
            self.AutoInputs = ecfg['AutoInputs']
            rospy.set_param("AutoInputs", str(self.AutoInputs))
        if "start" in ecfg:
            rospy.loginfo("Starting PYMLAB ROS servre with: %s" %(repr(self.devices)))
            self.status = True
            rate = rospy.Rate(self.rate)
            AutoInputs = self.AutoInputs
            devices = self.devices
            #sender = rospy.Publisher('pymlab_data', SensorValues, queue_size=20)
            values = {}
            for x in AutoInputs:
                for y in AutoInputs[x]:
                    #print "AutoInputs >>", x, y, 
                    #print str(x)+"/"+str(y), str(x)+"/"+str(y)
                    values[str(x)+"/"+str(y)] = str(x)+"/"+str(y)
            rospy.set_param("values", values)
           # print "\n run \n\n"
            while True:
                '''
                print "\r",
                for x in AutoInputs:
                    for y in AutoInputs[x]:
                        while self.pymlab_read: pass
                        self.pymlab_read = True
                        data = getattr(self.devices[devices[x].name], y)()
                        self.pymlab_read = False
                        print x, "%.3f"%data, "||",
                        sender.publish(name=str(devices[x].name)+"/"+str(y), value=data)
                        #senderTest.publish(data)
                print "\r",
                '''
                if self.err_count > 100:
                    rospy.logwarn("restarting pymlab")
                    self.err_count = 0
                    for i, bus in enumerate(self.bus):
                        try:
                            rospy.loginfo("reloading pymlab bus:" + repr(bus))
                            self.init_bus(i2c = self.i2c[i], bus = bus)
                        except Exception, e:
                            rospy.logerr(e)
                        
                rate.sleep()
            return True

    def drive(self, cfg):       # zpracovava requesty ostatatich nodu
        parameters = cfg.parameters
        method = cfg.method
        device = cfg.device
        print "parameters '%s', method '%s', device '%s'" %(cfg.parameters, cfg.method, cfg.device)
        while self.pymlab_read: time.sleep(0.01)
        self.pymlab_read = True
        try:
            if parameters == "" or parameters == None:
                reval = getattr(self.devices[cfg.device], cfg.method)()
            elif isinstance(eval(parameters), tuple):
                print "tuple:", eval(parameters)
                reval = getattr(self.devices[cfg.device], cfg.method)(*eval(parameters))
            else:
                reval = getattr(self.devices[cfg.device], cfg.method)(eval(parameters))
            self.pymlab_read = False
            return str(reval)
        except Exception, e:
            self.err_count += 1
            rospy.logerr("#01:"+repr(e))
            self.pymlab_read = False
            return str(False)


def main():
    ps = pymlab_server()
    rospy.init_node('pymlab_node')

    rospy.Subscriber("pymlab_server", PymlabServerStatusM, ps.status)
    s1 = rospy.Service('pymlab_init', PymlabInit, ps.init)
    s2 = rospy.Service('pymlab_server', PymlabServerStatus, ps.status)
    s3 = rospy.Service('pymlab_drive', PymlabDrive, ps.drive)               # slouzi k prijmuti requestu

    rospy.loginfo("Ready to get work.")
    rospy.spin()

if __name__ == "__main__":
    main()