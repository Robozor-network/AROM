#!/usr/bin/env python

import math
import time
import rospy
import std_msgs
import actionlib
from std_msgs.msg import String
from std_msgs.msg import Float32
from arom.srv import *
from arom.msg import *
import arom
import serial


class mount():
    def __init__(self, parent = None, name = "EQmod", port="/dev/ttyUSB0", connect = 1):
        self.init()
        self.slewing = False
        self.Autoconnect = connect
        self.port = port
        self.parent = parent
        self.name = name
        self.coordinates = [0, 0]  ## Ra, Dec
        self.coordinates_offset = [0, 0]
        self.coordinates_offset = [0, 0]
        self.coordinates_target = [0, 0]  ## Ra, Dec
        self.coordinates_geo = [0, 0, 0] ## Lat, Lon, Alt
        self.DayLenght = {'solar': (24.0*60*60), 'sideral': (23.0*60*60 + 56.0*60 + 4.0916), 'moon':  (24.0*60*60 + 49.0*60), 'custom': (24.0*60*60)}


        s_RegisterDriver = rospy.Service('driver/mount/%s' %self.name, arom.srv.DriverControl, self.reset)

        ##
        ##  Ceka to na spusteni AROMbrain nodu
        ##

        rospy.init_node('AROM_mount')
        rospy.loginfo("%s: wait_for_service: 'arom/RegisterDriver'" % self.name)
        rospy.wait_for_service('arom/RegisterDriver')
        rospy.loginfo("%s: >> brain found" % self.name)

        ##
        ##  Registrace zarizeni
        ##  >Arom returns 1 - OK, 0 - False
        ##

        RegisterDriver = rospy.ServiceProxy('arom/RegisterDriver', arom.srv.RegisterDriver)
        registred = RegisterDriver(name = self.name, sname= self.name, driver = 'EQmod', device = 'mount', status = 1)
        rospy.loginfo("%s: >> register %s driver: %s" %(self.name, 'EQmod', registred))

        ##
        ##  Spusti se Action servis pro zmenu cile
        ##

        self.act = actionlib.SimpleActionServer('arom/mount/%s/target'% self.name, arom.msg.MountTargetAction, execute_cb=self.ReciveTarget, auto_start = False)
        self.act.start()

        ##
        ##  Ovladac se pripoji k montazi
        ##

        if self.Autoconnect:
            self.connect()

        ##
        ##  Vytvoreni servisu na praci s montazi
        ##

        self.s_MountParameters = rospy.Service('arom/mount/parameter', arom.srv.MountParameter, self.MountParameter)
        rospy.Subscriber("arom/mount/", arom.msg.DriverControlm, self.reset)

        ##
        ##  Ovladac pujde ukoncit
        ##

        print "Mount loaded"
        rare = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.coordinates != self.coordinates_target and not self.slewing:
                self.slew()
            self.getPosition
            rospy.loginfo("loc: %s" %(str(self.coordinates)))
            rare.sleep()

    def reset(self, msg):
        rospy.loginfo(msg)
        data = eval(msg.data)
        if msg.type == 'Slew':
            self.setTarget([int(self._degToAxis(data['ra'])), int(self._degToAxis(data['dec']))])

    def MountParameter(self, MountParameter = None):
        rospy.loginfo('%s: GetNewParameters: %s' %(self.name, MountParameter))
        out = getattr(self, str(MountParameter.name))()
        rospy.loginfo('%s: GetNewParameters: out >> %s' %(self.name, str(out)))
        return arom.srv.MountParameterResponse(str(out), str('raw'), 1)

    def ReciveTarget(self, target = None):
        raise NotImplementedError()

    def reciveMSG(self, msg):
        raise NotImplementedError()

    def connect(self, param = None):
        raise NotImplementedError()

    def park(self, param = None):
        raise NotImplementedError()
        
    def unpark(self, param = None):
        raise NotImplementedError()
        
    def setpark(self, param = None):
        raise NotImplementedError()
        
    def getpark(self, param = None):
        raise NotImplementedError()

    def start(self, param = None):
        raise NotImplementedError()
        
    def slew(self, param = None):
        raise NotImplementedError()

    def track(self, param = None):
        raise NotImplementedError()

    def slew(self, param = None):
        raise NotImplementedError()

    def setPosition(self, param = None):
        raise NotImplementedError()

    def getPosition(self, param = None):
        raise NotImplementedError()

    def setPosition_geo(self, param = None):
        raise NotImplementedError()

    def getPosition_geo(self, param = None):
        raise NotImplementedError()

    def setTime(self, param = None):
        raise NotImplementedError()

    def getTime(self, param = None):
        print "position :)", param

    def getStatus(self, param = None):
        raise NotImplementedError()

    def setAligmentPoint(self, param = None):
        raise NotImplementedError()

    def getAligmentPoint(self, param = None):
        raise NotImplementedError()

    def setLimits(self, param = None):
        raise NotImplementedError

    def getLimits(self, param = None):
        raise NotImplementedError()

    def getDriverVer(self, param = None):
        print "version is blablabla"

    def setCoordinates(self, coordinates):    # in degrees
        self.coordinates = [self._degToAxis(coordinates[0], 0), self._degToAxis(coordinates[1], 1)]
        return self.coordinates

    def setTarget(self, coordinates):    # in degrees
        self.coordinates_target = [self._degToAxis(coordinates[0], 0), self._degToAxis(coordinates[1], 1)]
        return self.coordinates_target

    def getCoordinates(self):    # in degrees
        self.coordinates = [self._degToAxis(coordinates[0], 0), self._degToAxis(coordinates[1], 1)]
        return [self._axisPosToReal(self.coordinates[0], 0), self._axisPosToReal(self.coordinates[1], 1)]




############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################

######################################################################################
######################################################################################
##                                                                                  ##
##                     Driver for --EQmod-- mount controller                        ##
##                 ============================================                     ##
##                                                                                  ##
##                                                                                  ##
######################################################################################
        
class EQmod(mount):
    def init(self):   
        self.mountParams = {} 
        self.stepsPerRev={1:1, 2:1}
        self.data = ""  
        self.Initialize ='F'
        self.InquireMotorBoardVersion='e'
        self.InquireGridPerRevolution='a'
        self.InquireTimerInterruptFreq='b'
        self.InquireHighSpeedRatio='g'
        self.InquirePECPeriod='s'
        self.InstantAxisStop='L'
        self.NotInstantAxisStop='K'
        self.SetAxisPosition='E'
        self.GetAxisPosition='j'
        self.GetAxisStatus='f'
        self.SetSwitch='O'
        self.SetMotionMode='G'
        self.SetGotoTargetIncrement='H'      ##
        self.SetBreakPointIncrement='M'      ##
        self.SetBreakSteps='U'
        self.SetStepPeriod='I'                 ## 
        self.StartMotion='J'
        self.GetStepPeriod='D' # See Merlin protocol http://www.papywizard.org/wiki/DevelopGuide
        self.ActivateMotor='B' # See eq6direct implementation http://pierre.nerzic.free.fr/INDI/
        self.SetGuideRate='P'  # See EQASCOM driver
        self.Deactivate='d'
        self.Axis1='1'       # RA/AZ
        self.Axis2='2'       # DE/ALT
        self.AxRa=0          # RA/AZ
        self.AxDec=1         # DE/ALT
        self.AxisTick = 16777215.0
        self.DayLenght = {'solar': (24.0*60*60), 'sideral': (23.0*60*60 + 56.0*60 + 4.0916), 'moon':  (24.0*60*60 + 49.0*60), 'custom': (24.0*60*60)}
        self.coordinates = [0,0]
        self.coordinates_D = [0,0]
        self.syncOffSet = [0,0]

        ##
        ##  Registrace ovladace jako mount
        ##


    def _axisPosToDeg(self, loc, ax=0):
        return loc / self.stepsPerRev[ax+1]/360.0

    def _axisPosToReal(self, loc, ax=0):
        return loc / self.stepsPerRev[ax+1]/360.0 + self.syncOffSet[ax]

    def _degToAxis(self, loc, ax=0):
        return loc * self.stepsPerRev[ax+1]/360.0

    def _degRealToAxis(self, loc, ax=0):
        return loc * self.stepsPerRev[ax+1]/360.0 + self.syncOffSet[ax]

    def connect(self, port="/dev/ttyUSB0"):
        print "connect > start"
        rospy.loginfo("connect > start")
        
        if port:
            self.port = port

        self.ser = serial.Serial()
        self.ser.baudrate = 9600
        self.ser.port = self.port
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = 0 ## non blocking
        self.ser.open()
        self.mountParams['F1'] = self._GetData(self.Initialize, self.Axis1)
        self.mountParams['F2'] = self._GetData(self.Initialize, self.Axis2)
        self.mountParams['e'] = self._GetData(self.InquireMotorBoardVersion, self.Axis1)
        self.mountParams['a1'] = self._GetData(self.InquireGridPerRevolution, self.Axis1)
        self.mountParams['b1'] = self._GetData(self.InquireTimerInterruptFreq, self.Axis1)
        self.mountParams['g1'] = self._GetData(self.InquireHighSpeedRatio, self.Axis1)
        self.mountParams['a2'] = self._GetData(self.InquireGridPerRevolution, self.Axis2)
        self.mountParams['b2'] = self._GetData(self.InquireTimerInterruptFreq, self.Axis2)
        self.mountParams['g2'] = self._GetData(self.InquireHighSpeedRatio, self.Axis2)
        print "Connection DONE"
        self.getPosition()
        self.coordinates_target = self.coordinates

    # Get mount name
        self.electronicVersion={}
        self.electronicVersion['raw']=self.Revu24str2long(self.mountParams['e']);
        self.electronicVersion['mountID'] = ((self.electronicVersion['raw'] & 0xFF) << 16) | ((self.electronicVersion['raw'] & 0xFF00)) | ((self.electronicVersion['raw'] & 0xFF0000) >> 16);
        self.mountID=self.electronicVersion['mountID'] & 0xFF;

        if  self.electronicVersion ['mountID'] & 0xFF ==  0x00:
            self.electronicVersion['name'] = "EQ6"

        elif self.electronicVersion['mountID'] & 0xFF == 0x01:
            self.electronicVersion['name'] = "HEQ5"

        elif self.electronicVersion['mountID'] & 0xFF == 0x02:
            self.electronicVersion['name'] = "EQ5"

        elif self.electronicVersion['mountID'] & 0xFF == 0x03:
            self.electronicVersion['name'] = "EQ3"

        elif self.electronicVersion['mountID'] & 0xFF == 0x80:
            self.electronicVersion['name'] = "GT"

        elif self.electronicVersion['mountID'] & 0xFF == 0x81:
            self.electronicVersion['name'] = "MF"

        elif self.electronicVersion['mountID'] & 0xFF == 0x82:
            self.electronicVersion['name'] = "114GT"

        elif self.electronicVersion['mountID'] & 0xFF == 0x90:
            self.electronicVersion['name'] = "DOB"

        elif self.electronicVersion['mountID'] & 0xFF == 0xF0:
            self.electronicVersion['name'] = "GEEHALEL"
        else:
            self.electronicVersion['name'] = "Unknown_0x%x" %(self.electronicVersion['mountID'] & 0xFF)

    # get Steps per one axcis revolution
        self.stepsPerRev={}
        self.stepsPerRev[1]= self.Revu24str2long(self.mountParams['a1'])
        self.stepsPerRev[2]= self.Revu24str2long(self.mountParams['a1'])

    def slew(self, target = [None, None]):
        self.setPosition(target)



    def _GetData(self, cmd, axis, param = None, max_time = 10):
        #rospy.loginfo("GetDataRequest %s" %str(cmd))
        SkywatcherLeadingChar = ':'
        #SkywatcherTrailingChar = 0x0d
        SkywatcherTrailingChar = '\r'

        if not param:
            req = "%c%c%c%c" %(SkywatcherLeadingChar, cmd, axis, SkywatcherTrailingChar)
        else:
            req = "%c%c%c%s%c" %(SkywatcherLeadingChar, cmd, axis, param, SkywatcherTrailingChar)

        rospy.loginfo("Send data: '%s' >%s" %(str(cmd), repr(req)))
        self.ser.write(req)
        data = ""
        while 1:
            data = self.ser.read(999)
            if len(data) > 0:
                pass
            if ("#" in data) or ("=" in data) or ("!" in data):
                rospy.loginfo("Recived data:\t \t >>> %s" %(repr(data)))
                break
            time.sleep(0.05)

        if "=" in data:
            return data
        else:
            return data

    def ReciveTarget(self, target = None):
        rospy.loginfo("RecivedNewTarget %s" %(repr(target)))

    def setPosition(self, loc = [None,None], param = None):
        if loc != [None, None]:
            self.coordinates_target = loc
        self.getPosition()
        change = [0,0]
        change[0] = self.coordinates[0] - self.coordinates_target[0]
        change[1] = self.coordinates[1] - self.coordinates_target[1]

        dirRA = '21'
        dirDEC= '21'
        if change[0] < 0:
            dirRA = '20'
        if change[1] < 0:
            dirDEC = '20'

        rospy.loginfo("SLEW TO %s (%s) with difference %s" %(self.coordinates, self.getCoordinates, change))

        stepsRA = self.stepsPerRev[1] * change[0] / 360.0
        stepsDEC = self.stepsPerRev[2] * change[1] / 360.0

        #stepsRA = self._degRealToAxis(change[0], self.AxRa)
        #stepsDEC = self._degRealToAxis(change[1], self.AxDec)

        try:
            ra = self._GetData('K', self.Axis1, "")
            ra = self._GetData('K', self.Axis2, "")

            ra = self._GetData(self.NotInstantAxisStop, self.Axis1,)
            ra = self._GetData(self.GetAxisStatus, self.Axis1,)
            ra = self._GetData(self.SetMotionMode, self.Axis1, dirRA)
            ra = self._GetData(self.GetAxisStatus, self.Axis1)
            ra = self._GetData(self.SetStepPeriod, self.Axis1, "120000")
            ra = self._GetData(self.SetGotoTargetIncrement, self.Axis1, self.long2Revu24str(stepsRA))
            ra = self._GetData(self.SetBreakPointIncrement, self.Axis1, "C80000")
           
            ra = self._GetData(self.NotInstantAxisStop, self.Axis2)
            ra = self._GetData(self.GetAxisStatus, self.Axis2)
            ra = self._GetData(self.SetMotionMode, self.Axis2, dirDEC)
            ra = self._GetData(self.GetAxisStatus, self.Axis2)
            ra = self._GetData(self.SetStepPeriod, self.Axis2, "120000")
            ra = self._GetData(self.SetGotoTargetIncrement, self.Axis2, self.long2Revu24str(stepsDEC))
            ra = self._GetData(self.SetBreakPointIncrement, self.Axis2, "C80000")

            ra = self._GetData(self.StartMotion, self.Axis1)
            ra = self._GetData(self.StartMotion, self.Axis2)

            time.sleep(1)
            while True:
                ax0 = self._GetData(self.GetAxisStatus, self.Axis1)
                ax1 = self._GetData(self.GetAxisStatus, self.Axis2)
                #print "osy", ax1, ax0
                if ax0[2] ==  "0" and ax1[2] ==  "0":
                    self.coordinates = self.coordinates_target
                    ra = self._GetData(self.NotInstantAxisStop, self.Axis1)
                    ra = self._GetData(self.NotInstantAxisStop, self.Axis2)
                    sp = self.long2Revu24str( int(self.stepsPerRev[1]/self.DayLenght['sideral'])*6)
                    ra = self._GetData(self.SetStepPeriod, self.Axis1, sp) 
                    ra = self._GetData(self.StartMotion, self.Axis1)
                    break
                time.sleep(0.1)
            
            sp = self.long2Revu24str( int(self.stepsPerRev[1]/self.DayLenght['sideral'])*6)
            ra = self._GetData(self.SetStepPeriod, self.Axis1, sp) 
            ra = self._GetData(self.StartMotion, self.Axis1)

        except Exception, e:
            print e
        print "coordinates, ra:%s, dec:%s" %(str(self.coordinates[0]), str(self.coordinates[1]))

        return self.coordinates_target

    def getPosition(self, param = None):
        try:
            raw = self._GetData(self.GetAxisPosition, self.Axis1)
            decw = self._GetData(self.GetAxisPosition, self.Axis2)
        except Exception, e:
            print e
        ra = self.Revu24str2long(raw)
        dec = self.Revu24str2long(decw)
        rospy.set_param("mount/ra", ra)
        rospy.set_param("mount/dec", dec)
        
        self.coordinates = [ra, dec]
        self.D_coordinates = [self._axisPosToDeg(ra, self.AxRa), self._axisPosToDeg(dec, self.AxDec)]
        return self.coordinates

    def getRealPosition(self, param = None):
        self.getPosition()
        self.coordinates_real = (self.coordinates[0]+self.syncOffSet[0], self.coordinates[1]+self.syncOffSet[1])
        return self.coordinates_real

    def GetMotorStatus(self, axis):
        response = self._GetData(self.GetAxisStatus, axis);
        #print response, response[2], response[3]
        if axis == self.Axis1:
            RAInitialized=(response[3]&0x01)
            RARunning=(response[2]&0x01)
            if (response[1] & 0x01 ):
                RAStatus.slewmode=SLEW
            else:
                RAStatus.slewmode=GOTO
            if (response[1] & 0x02 ):
                RAStatus.direction=BACKWARD
            else:
                RAStatus.direction=FORWARD
            if (response[1] & 0x04 ):
                RAStatus.speedmode=HIGHSPEED
            else:
                RAStatus.speedmode=LOWSPEED

        elif axis == self.Axis2:
            DEInitialized=(response[3]&0x01)
            DERunning=(response[2]&0x01)
            if (response[1] & 0x01 ):
                DEStatus.slewmode=SLEW
            else:
                DEStatus.slewmode=GOTO
            if (response[1] & 0x02 ):
                DEStatus.direction=BACKWARD
            else:
                DEStatus.direction=FORWARD
            if (response[1] & 0x04 ):
                DEStatus.speedmode=HIGHSPEED
            else:
                DEStatus.speedmode=LOWSPEED
        else:
            pass


    def Revu24str2long(self, s):
        s = str(s)[1:-1]
        res = 0
        try:
            res = int(s[4], 16)
            res <<= 4
            res |= int(s[5], 16)        
            res <<= 4
            res |= int(s[2], 16)        
            res <<= 4
            res |= int(s[3], 16)        
            res <<= 4
            res |= int(s[0], 16)        
            res <<= 4
            res |= int(s[1], 16)  
        except Exception, e:
            print "chyba v Revu24str2long: s a res je:", s
        return res

    def long2Revu24str(self, n):
        n = int(n)
        out=""
        hexa = ['0', '1', '2', '3', '4', '5', '6', '7', 
                '8', '9', 'A', 'B', 'C', 'D', 'E', 'F']
        out += hexa[(n & 0xF0) >> 4]
        out += hexa[(n & 0x0F)]
        out += hexa[(n & 0xF000) >> 12]
        out += hexa[(n & 0x0F00) >> 8]
        out += hexa[(n & 0xF00000) >> 20]
        out += hexa[(n & 0x0F0000) >> 16]
        out += '\0'
        print out
        return out

'''

    Skywatcher::Highstr2long(char *s)
        res = 0
        res =HEX(s[0]); res <<= 4
        res|=HEX(s[1])
        return res

'''


############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################
############################################################################################################################################################################

######################################################################################
######################################################################################
##                                                                                  ##
##                   Driver for SynScan mount hand controller                       ##
##                 ============================================                     ##
##                                                                                  ##
##      It does not support telescope setAligmentPoint                              ##
##                                                                                  ##
######################################################################################

class SynScan(mount):

    def connect(self, port="/dev/ttyUSB0"):
        print "connect > start"
        rospy.loginfo("connect > start")
        
        if port:
            self.port = port

        self.ser = serial.Serial()
        self.ser.baudrate = 9600
        self.ser.port = self.port
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = 0 ## non blocking
        self.ser.open()

    def _GetData(self, string, max_time = 10):
        data = ''
        self.ser.write(string)
        while not "#" in data:
            data = self.ser.read(999)
            if len(data) > 0:
                return data
            time.sleep(0.05)
        return data

    def getPosition(self, param = None):
        ra, dec = 0, 0
        self._GetData('e')
        raw = self._GetData('e')                    # nejaka chyba ve vycitani ... spravny 
        raw = str(raw).split('#')[0].split(',')
        ra = int(raw[0],16)*360.0/4294967296.0
        dec = int(raw[1],16)*360.0/4294967296.0
        self.coordinates = [ra, dec]
        return self.coordinates

    def setPosition_geo(self, param = None):
        raise NotImplementedError()

    def getPosition_geo(self, param = None):
        print "position :)", param

    def setTime(self, param = None):
        raise NotImplementedError()

    def getTime(self, param = None):
        Time_Q = Time_R = Time_S = Time_T = Time_U = Time_V = Time_W = Time_X = 0
        time_data = {}
        raw = self._GetData('h')
        print repr(raw)
        raw = repr(self._GetData('h').decode('ascii'))               # nejaka chyba ve vycitani ... spravny
        print raw
        raw1 = raw.split("#")[0]
        print raw1
        raw2 = raw1.split('\\x')
        print raw2
        for x in xrange(1,len(raw2)):
            try:
                print x, int(raw2[x],16)
            except Exception, e:
                raise e
        
        return str([''])

        

if __name__ == '__main__':
    mount = EQmod()
    #mount.connect()
    mount.getPosition()
    #for x in xrange(1,3):
    #    time.sleep(0.5)
    #    pos = mount.getPosition()
    ra = pos[0]
    dec = pos[1]
    ra += 10
    dec += 10
    mount.setPosition([ra, dec])
    #print "-----------------------------"
    #ra = ra - 10
    #dec = dec - 10
    #mount.setPosition([ra, dec])

    #while 1:
    #   time.sleep(1)
    #   # mount.getPosition()
