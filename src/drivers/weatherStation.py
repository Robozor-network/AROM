#!/usr/bin/env python

import math
import time
import rospy
import std_msgs
import actionlib
import json
from std_msgs.msg import String
from std_msgs.msg import Float32
from arom.srv import *
from arom.msg import *
import numpy as np
import serial
from astropy.time import Time
import astropy.units as u
import MySQLdb as mdb

class weatherStation(object):
    def __init__(self, parent = None, arg = None, name = "AWS01B", port="", connect = True, var = {}):
        self.arg = arg
        self.Autoconnect = connect
        self.port = port
        self.parent = parent
        self.name = self.arg['name']
        self.sname = self.name
        self.variables = var
        
        ##
        ##  Pripojeni k databazi
        ##

        self.connection = mdb.connect(host="localhost", user="root", passwd="root", db="AROM", use_unicode=True, charset="utf8")
        self.cursorobj = self.connection.cursor()

        ##
        ##  Inicializace vlastniho ovladace
        ##

        self.init()

        s_RegisterDriver = rospy.Service('driver/weatherStation/%s' %self.name, arom.srv.DriverControl, self.reset)

        ##
        ##  Ceka to na spusteni AROMbrain nodu
        ##

        rospy.init_node('AROM_weatherStation')
        rospy.loginfo("%s: wait_for_service: 'arom/RegisterDriver'" % self.name)
        rospy.wait_for_service('arom/RegisterDriver')
        rospy.loginfo("%s: >> brain found" % self.name)

        ##
        ##  Registrace zarizeni
        ##  >Arom returns 1 - OK, 0 - False
        ##

        RegisterDriver = rospy.ServiceProxy('arom/RegisterDriver', arom.srv.RegisterDriver)
        registred = RegisterDriver(name = self.name, sname = self.name, driver = self.arg['driver'], device = self.arg['type'], service = 'arom/driver/%s/%s' %(self.arg['type'], self.name), status = 1)
        rospy.loginfo("%s: >> register %s driver: %s" %(self.name, 'AWS01A', registred))


        ##
        ##  Ovladac se pripoji k montazi
        ##

        if self.Autoconnect:
            self.connect()

        ##
        ##  Ovladac pujde ukoncit
        ##

        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            try:
                data = self.mesure()
                self.datalog(data)
            except Exception, e:
                rospy.logerr(e)
            rate.sleep()


        self.connection.close()

    def _sql(self, query, read=False):
        #print query
        result = None
        try:
            self.cursorobj.execute(query)
            result = self.cursorobj.fetchall()
            if not read:
                self.connection.commit()
        except Exception, e:
            rospy.logerr("MySQL: %s" %repr(e))
        return result

    def reset(self, val=None):
        pass

    def datalog(self, val = []):
        for row in val:
            #self._sql("INSERT INTO `AROM`.`weather` (`date`, `type_id`, `sensors_id`, `value`) VALUES ('%f', '%i', '%i', '%f');" % (row['time'], 0, 0, row['value']))
            self._sql("INSERT INTO `AROM`.`weather` (`date`, `type_id`, `sensors_id`, `value`) VALUES ('%f', %i, (SELECT sensors_id FROM sensors WHERE sensor_name = '%s'), '%f');" % (row['time'], 0, row['name'], row['value']))
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
        
class AWS01B(weatherStation):
    def init(self): 
        self.variables = {
            'AWS_LTS_temp': 0,
            'AWS_SHT_temp': 0,
            'AWS_SHT_humi': 0
            }
        rospy.loginfo("AWS01A weather station requires 'pymlab_drive' service from 'ROSpymlabServer' node")
        rospy.loginfo("run>> 'rosrun arom initPymlab.py'")
        rospy.wait_for_service('pymlab_drive')
        self.pymlab = rospy.ServiceProxy('pymlab_drive', PymlabDrive)
        rospy.loginfo("%s: >> 'ROSpymlabServer' found" % self.name)
        self.last_wind_mes = Time.now()


    def mesure(self):
        AWS_LTS_temp = eval(self.pymlab(device="sht25", method="get_temp_8bit", parameters=None).value)
        #AWS_LTS_hum = eval(self.pymlab(device="sht25", method="get_hum_8bit", parameters=None).value)
        self.variables['AWS_LTS_temp'] = AWS_LTS_temp
        TempHum = eval(self.pymlab(device="AWS_humi", method="get_TempHum", parameters=None).value)
        #print TempHum
        self.variables['AWS_AMBIENT_temp'] = TempHum[0]
        self.variables['AWS_AMBIENT_humi'] = TempHum[1]
        #print self.variables
        rospy.set_param("weatherStation", str(self.variables))
        rospy.loginfo('LTS: %s, sht31.temp: %s, sht31.humi: %s' %(str(AWS_LTS_temp), str(TempHum[0]), str(TempHum[1])))

        if self.last_wind_mes + 10 * u.s < Time.now():
            angles = np.zeros(5)
            angles[4] = eval(self.pymlab(device="AWS_wind_s", method="get_angle", parameters='').value)
            time.sleep(0.01)
            angles[3] = eval(self.pymlab(device="AWS_wind_s", method="get_angle", parameters='').value)
            time.sleep(0.01)
            angles[2] = eval(self.pymlab(device="AWS_wind_s", method="get_angle", parameters='').value)
            time.sleep(0.01)
            angles[1] = eval(self.pymlab(device="AWS_wind_s", method="get_angle", parameters='').value)
            n = 0
            speed = 0
            AVERAGING = 50

            for i in range(AVERAGING):
                time.sleep(0.01)
                angles[0] = eval(self.pymlab(device="AWS_wind_s", method="get_angle", parameters='').value)
                
                if (angles[0] + n*360 - angles[1]) > 300:
                    n -= 1
                    angles[0] = angles[0] + n*360

                elif (angles[0] + n*360 - angles[1]) < -300:  # compute angular speed in backward direction.
                    n += 1
                    angles[0] = angles[0] + n*360

                else:
                    angles[0] = angles[0] + n*360
                
                speed += (-angles[4] + 8*angles[3] - 8*angles[1] + angles[0])/12
                angles = np.roll(angles, 1)

            #speed = speed/AVERAGING             # apply averaging on acummulated value.
            
            self.last_wind_mes = Time.now()
            return [{'value':AWS_LTS_temp, 'name':'AWS_telescope_temp_lts', 'guantity': 'C', 'time': Time.now().unix},
                    {'value':TempHum[0], 'name':'AWS_ambient_temp_2m', 'guantity': 'C', 'time': Time.now().unix},
                    {'value':TempHum[1], 'name':'AWS_ambient_humi', 'guantity': 'perc', 'time': Time.now().unix},
                    {'value':speed, 'name':'AWS_ambient_wind_speed_5m', 'guantity': 'ms-1', 'time': Time.now().unix}]


        return [{'value':AWS_LTS_temp, 'name':'AWS_telescope_temp_lts', 'guantity': 'C', 'time': Time.now().unix},
                {'value':TempHum[0], 'name':'AWS_ambient_temp_2m', 'guantity': 'C', 'time': Time.now().unix},
                {'value':TempHum[1], 'name':'AWS_ambient_humi', 'guantity': 'perc', 'time': Time.now().unix}]
        

    def connect(self):
        pass



if __name__ == '__main__':
    cfg = rospy.get_param("ObservatoryConfig/file")
    with open(cfg) as data_file:
        config = json.load(data_file)
    for x in config:
        if x['name'] == sys.argv[1]:
            break
    weatherStation = locals()[x['driver']](arg = x)
    #weatherStation = AWS01B()