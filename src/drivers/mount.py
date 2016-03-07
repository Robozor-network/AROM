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
    def __init__(self, parent = None, name = "mount", port="/dev/ttyUSB0", connect = 1):
        self.Autoconnect = connect
        self.port = port
        self.parent = parent
        self.name = name
        self.coordinates = [0, 0]  ## Ra, Dec
        self.coordinates_offset = [0, 0]
        self.coordinates_target = [0, 0]  ## Ra, Dec
        self.coordinates_geo = [0, 0, 0] ## Lat, Lon, Alt

        ##
        ##  Ceka to na spusteni AROMbrain nodu
        ##

        rospy.init_node('AROM_mount')
        rospy.loginfo("%s: wait_for_service: 'arom/RegisterDriver'" % self.name)
        rospy.wait_for_service('arom/RegisterDriver')
        rospy.loginfo("%s: >> done" % self.name)

        ##
        ##  Registrace zarizeni
        ##  >Arom returns 1 - OK, 0 - False
        ##

        RegisterDriver = rospy.ServiceProxy('arom/RegisterDriver', arom.srv.RegisterDriver)
        registred = RegisterDriver(name = self.name, driver = 'SynScan', status = 1)
        rospy.loginfo("%s: >> register %s" %(self.name, registred))

        ##
        ##  Spusti se Action servis pro zmenu cile
        ##

        self.act = actionlib.SimpleActionServer('AROM/mount/target', arom.msg.MountTargetAction, execute_cb=self.ReciveTarget, auto_start = False)
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

        ##
        ##  Ovladac pujde ukoncit
        ##

        rospy.spin()


    def MountParameter(self, MountParameter = None):
        rospy.loginfo('%s: GetNewParameters: %s' %(self.name, MountParameter))
        out = getattr(self, str(MountParameter.name))()
        rospy.loginfo('%s: GetNewParameters: out >> %s' %(self.name, str(out)))
        return arom.srv.MountParameterResponse(str(out), str('raw'), 1)

    def ReciveTarget(self, target = None):
        print "Target:", target
        rospy.loginfo('%s: GetNewTarget: %s' %(self.name, target))

    def reciveMSG(self, msg):
        pass

    def connect(self, param = None):
        print "connecting"

    def park(self, param = None):
        print "park"
        
    def unpark(self, param = None):
        print "UNpark"
        
    def setpark(self, param = None):
        print "SENpark"
        
    def getpark(self, param = None):
        print "GENpark"

    def start(self, param = None):
        print "EQmod driver started"
        
    def slew(self, param = None):
        raise NotImplementedError()

    def track(self, param = None):
        raise NotImplementedError()

    def setPosition(self, param = None):
        raise NotImplementedError()

    def getPosition(self, param = None):
        print "position :)", param

    def setPosition_geo(self, param = None):
        raise NotImplementedError()

    def getPosition_geo(self, param = None):
        print "position :)", param

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
        

class EQmod(mount):

    ##
    ##  to slew use ":G100", ":I1060000", ":H17C028C", ":M1800C00"
    ##                motion mode    stepperperiod   SetGoToTargetIncrement  SetBreakPointIncrement
    ## 

    def __init__(self):    
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
        self.AxisTick = 16777215.0


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
        print self._GetData(self.InquireMotorBoardVersion, self.Axis1)
        print self._GetData(self.InquireGridPerRevolution, self.Axis1)
        print self._GetData(self.InquireTimerInterruptFreq, self.Axis1)
        print self._GetData(self.InquireHighSpeedRatio, self.Axis1)
        print self._GetData(self.InquireGridPerRevolution, self.Axis2)
        print self._GetData(self.InquireTimerInterruptFreq, self.Axis2)
        print self._GetData(self.InquireHighSpeedRatio, self.Axis2)

    def _GetData(self, cmd, axis, param = None, max_time = 10):
        rospy.loginfo("GetDataRequest %s" %str(cmd))
        SkywatcherLeadingChar = ':'
        #SkywatcherTrailingChar = 0x0d
        SkywatcherTrailingChar = '\r'

        if not param:
            req = "%c%c%c%c" %(SkywatcherLeadingChar, cmd, axis, SkywatcherTrailingChar)
            print "##>%s" %repr(req)
        else:
            req = "%c%c%c%s%c" %(SkywatcherLeadingChar, cmd, axis, param, SkywatcherTrailingChar)
            print "##>%s" %repr(req)
        self.ser.write(req)
        data = ""
        while 1:
            data = self.ser.read(999)
            if len(data) > 0:
                pass
            if ("#" in data) or ("=" in data) or ("!" in data):
                print "++>%s" %repr(data)
                break
            time.sleep(0.05)

        if "#" in data:
            print "<<>> print #"
        elif "=" in data:
            #print "<<>> print ="
            return data
        elif "!" in data:
            print "<<>> print !"
        else:
            print "<<>> poblem"
            return data

    def setPosition(self, loc = [0,0], change = [0,0], param = None):
        self.coordinates_target = loc
        change[0] = self.coordinates[0] - self.coordinates_target[0]
        change[1] = self.coordinates[1] - self.coordinates_target[1]
        try:
            ra = self._GetData('K', self.Axis1, "")
            ra = self._GetData('K', self.Axis2, "")
            '''
            :G201<cr>
            :f2<cr>
            :I2060000<cr>
            :H2EB087D<cr>
            :M2800C00<cr>
            '''
            ra = self._GetData(self.NotInstantAxisStop, self.Axis1,)
            ra = self._GetData(self.GetAxisStatus, self.Axis1,)
            ra = self._GetData('G', self.Axis1, "21")
            ra = self._GetData('f', self.Axis1, "")
            ra = self._GetData('I', self.Axis1, "120000")        # J
            ra = self._GetData('H', self.Axis1, "E24100")
            ra = self._GetData('M', self.Axis1, "C80000")
            ra = self._GetData('J', self.Axis1)
           
            '''
            2016-03-07T18:48:54: dispatch_command: ":J1", 4 bytes written 
            2016-03-07T18:48:54: read_eqmod: "=", 2 bytes read 
            2016-03-07T18:48:54: dispatch_command: ":M1C80000", 10 bytes written 
            2016-03-07T18:48:54: read_eqmod: "=", 2 bytes read 
            2016-03-07T18:48:54: dispatch_command: ":H1E24100", 10 bytes written 
            2016-03-07T18:48:54: read_eqmod: "=", 2 bytes read 
            2016-03-07T18:48:54: dispatch_command: ":I1120000", 10 bytes written 
            2016-03-07T18:48:54: read_eqmod: "=201", 5 bytes read 
            2016-03-07T18:48:54: dispatch_command: ":f1", 4 bytes written 
            2016-03-07T18:48:54: read_eqmod: "=", 2 bytes read 
            2016-03-07T18:48:54: dispatch_command: ":G121", 6 bytes written 
            '''
            ra = self._GetData(self.NotInstantAxisStop, self.Axis2)
            ra = self._GetData(self.GetAxisStatus, self.Axis2)
            ra = self._GetData('G', self.Axis2, "21")
            ra = self._GetData('f', self.Axis2)
            ra = self._GetData('I', self.Axis2, "120000")        # J
            ra = self._GetData('H', self.Axis2, "E24100")
            ra = self._GetData('M', self.Axis2, "C80000")
            ra = self._GetData('J', self.Axis2)



            #dec = self._GetData(self.SetBreakPointIncrement, self.Axis2, "800C00")
            #ra = self._GetData('M', self.Axis1, "01")
            #dec = self._GetData('M', self.Axis2, "01")
            #print self._GetData(self.StartMotion,1)
            #print self._GetData(self.StartMotion,2)
        except Exception, e:
            print e
        print ra, dec, "   eaeoeaoeoae"

        return self.coordinates_target

    def getPosition(self, param = None):
        #ra  = self.coordinates[0]
        #dec = self.coordinates[1]
        #print self.coordinates
        try:
            raw = self._GetData(self.GetAxisPosition, self.Axis1)
            decw = self._GetData(self.GetAxisPosition, self.Axis2)
        except Exception, e:
            print e
        ra = self.Revu24str2long(raw[1:-1])
        dec = self.Revu24str2long(decw[1:-1])
        print "%s: >> position >> RA:%s, DEC:%s - ra: %.5f dec: %.5f --- %s, $$$ %s %s" %("", str(ra), str(dec), float(ra/self.AxisTick*360.0), float(dec/self.AxisTick*360.0), self.long2Revu24str(ra), repr(raw), repr(decw))
        
        self.coordinates = [float(ra/self.AxisTick*360.0), float(dec/self.AxisTick*360.0)]
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

    def GetMotorStatus(self, axis):
        response = self._GetData(self.GetAxisStatus, axis);
        print response, response[2], response[3]
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
        s = str(s)
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
        n = int(n/360*self.AxisTick)
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

'''



    bool Skywatcher::dispatch_command(SkywatcherCommand cmd, SkywatcherAxis axis, char *command_arg) throw (EQModError)
{
  int err_code = 0, nbytes_written=0, nbytes_read=0;

  // Clear string
  command[0] = '\0';
  
  if (command_arg==NULL) 
    snprintf(command, SKYWATCHER_MAX_CMD, "%c%c%c%c", SkywatcherLeadingChar, cmd, axis, SkywatcherTrailingChar);
  else
    snprintf(command, SKYWATCHER_MAX_CMD, "%c%c%c%s%c", SkywatcherLeadingChar, cmd, axis, command_arg, SkywatcherTrailingChar);

  tcflush(fd, TCIOFLUSH);
  
  if  ( (err_code = tty_write_string(fd, command, &nbytes_written) != TTY_OK))
    {
      char ttyerrormsg[ERROR_MSG_LENGTH];
      tty_error_msg(err_code, ttyerrormsg, ERROR_MSG_LENGTH);
      throw EQModError(EQModError::ErrDisconnect, "tty write failed, check connection: %s", 
               ttyerrormsg);
      return false;
   }

  //if (INDI::Logger::debugSerial(cmd)) {
    command[nbytes_written-1]='\0'; //hmmm, remove \r, the  SkywatcherTrailingChar
    DEBUGF(DBG_COMM, "dispatch_command: \"%s\", %d bytes written", command, nbytes_written);
    debugnextread=true;
  //}
  return true;
}
'''
        

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
        rospy.loginfo("GetDataRequest %s" %str(string))
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
        

        '''
        time_param = ['Time_Q', 'Time_R', 'Time_S', 'Time_T', 'Time_U', 'Time_V', 'Time_W', 'Time_X']
        print raw
        for x in range(len(raw)):
            try:
                print x,
                print int(str(x),16),
                time_data[time_param[int(x,16)]] = raw[int(x,16)]
                print time_param[int(x,16)], raw[int(x,16)]
            except Exception, e:
                print e
        '''
        
        return str([''])

        

if __name__ == '__main__':
    mount = EQmod()
    mount.connect()
    mount.getPosition()
    #mount.GetMotorStatus(1)
    #mount.GetMotorStatus(2)
    for x in xrange(1,3):
        time.sleep(0.5)
        pos = mount.getPosition()
    print "aeoaoeaoeeo", pos
    ra = pos[0]
    dec = pos[1]
    ra += 1000
    dec += 1000
    print "aeoaoeaoeeo", pos
    mount.setPosition([ra, dec])

    while 1:
        time.sleep(0.5)
        mount.getPosition()
