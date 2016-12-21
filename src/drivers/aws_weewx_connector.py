
"""
   sudo ln aws_weewx_connector.py /usr/share/weewx/weewx/drivers/aws_arom.py
"""

import time
import logging

import weedb
import weewx.drivers
import weeutil.weeutil

import rospy
from arom.srv import *
from arom.msg import *

DRIVER_NAME = 'AWS_AROM'
DRIVER_VERSION = "0.1"


data = {}

def callback(recive):
    global data
    for i, type in enumerate(recive.type):
        data[type] = recive.value[i]
    print "-",

def loader(config_dict, engine):
    # This loader uses a bit of a hack to have the simulator resume at a later
    # time. It's not bad, but I'm not enthusiastic about having special
    # knowledge about the database in a driver, albeit just the loader.

    station = AWS_AROM(**config_dict)
    return station
        
class AWS_AROM(weewx.drivers.AbstractDevice):
    def __init__(self, **stn_dict):
        self.stn_dict = stn_dict
        print stn_dict[DRIVER_NAME]
        name = 'aws_weewx_connector'
        rospy.init_node(name, anonymous=True)
        rospy.Subscriber("/aws_out", msg_WeatherStation, callback)

    def getWeewxName(self, arom_name):
        table = {'temperatureAWS0': ("outTemp", 1),
                 'temperatureTEL0': ("inTemp", 1),
                 'humidityAWS0': ("outHumidity", 1),
                 'pressureAWS': ("pressure", 1),
                 'windspdAWS': ("windSpeed", 1),
                 'winddirAWS': ("windDir", 1),
                 'lightAWS': ("radiation", 0.0079),}
        return table.get(arom_name, None)


    def genLoopPackets(self):
        while data == {}:
            print "aaaaaaaaaaa"
            pass
        while True: 
            print data
            print "------------"
            try:
                _packet = {"dateTime": time.time(),
                           "usUnits" : weewx.METRICWX}
                print _packet

                for name in data:
                    weewx_id = self.getWeewxName(name)
                    if weewx_id:
                        print "---", name, weewx_id[0],data[name]*weewx_id[1]
                        _packet[weewx_id[0]] = data[name]*weewx_id[1]
                    else:
                        print "---", name, weewx_id

                print _packet
                yield _packet
            except Exception, e:
                print e
            time.sleep(5)
            
    
    @property
    def hardware_name(self):
        return "AWS_AROM"


def confeditor_loader():
    return AWSConfEditor()

class AWSConfEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
[AWS_AROM]
    # This section is for the AWS_AROM weather station

    # The driver to use:
    driver = weewx.drivers.arom_aws

    # Sensor mapping to the weewx data
    [[Sensor_mapping]]
        #outTemp = barometer, 1

"""


if __name__ == "__main__":
    station = AWS_AROM()
    for packet in station.genLoopPackets():
        print weeutil.weeutil.timestamp_to_string(packet['dateTime']), packet