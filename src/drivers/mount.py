#!/usr/bin/env python
import math
import time
import rospy
import std_msgs
import actionlib
#import pandas
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float32
from arom.srv import *
from arom.msg import *
import arom
import serial
import json
from astropy import units as u
from astropy.coordinates import SkyCoord
from astropy.coordinates import Angle
from astropy.coordinates import EarthLocation
from astropy.coordinates import AltAz
from astropy.time import Time
from astropy.time import TimeDelta


class mount():
    def __init__(self, parent = None, arg = None, name = "EQmod", port="/dev/ttyUSB0", connect = True):
        self.arg = arg
        self.observatory = {}
        self.mountParams = {}
        self.observatory['coordinates'] = EarthLocation(lat=49*u.deg, lon=14*u.deg, height=300*u.m)
        self.newTarget = 0 ## X
        self.slewing = False ## X
        self.tracking = False ## X
        self.trackingMode = 'sidreal' ## X
        self.Autoconnect = connect
        self.port = port
        self.parent = parent
        self.name = self.arg['name']
        self.sname = self.name

        self.mountParams['tracking'] = False
        self.mountParams['trackingOffTime'] = Time.now()
        self.mountParams['trackingMode'] = 'sidreal'
        self.mountParams['newTarget'] = 0
        self.mountParams['coordinates'] = SkyCoord(alt=0.0, az=90.0, frame='altaz', unit='deg', obstime = Time.now(), location = self.observatory['coordinates']).icrs
        if rospy.get_param('heq5/restoreloc', None):
            self.mountParams['coordinates'] = SkyCoord(rospy.get_param('heq5/restoreloc'), unit='deg', obstime = Time.now(), frame = 'altaz', location = self.observatory['coordinates']).icrs
            rospy.loginfo("restoring saved position %s" %(repr(self.mountParams['coordinates'])))
        self.mountParams['coordinates_target'] = self.mountParams['coordinates']

        self.init_time = Time.now()
        self.DayLenght = {
                        'solar': (24.0*60*60),
                        'sidreal': (23.0*60*60 + 56.0*60 + 4.0916),
                        'moon':  (24.0*60*60 + 49.0*60),
                        'custom': (24.0*60*60)
        }
        self._setHorizont()

##
##  Vlastni inicizalizace
        self.init()

                                                                                                ##  Ceka to na spusteni AROMbrain nodu
        rospy.init_node('AROM_mount')
        rospy.loginfo("%s: wait_for_service: '/arom/RegisterDriver'" % self.name)
        rospy.wait_for_service('/arom/RegisterDriver')
        rospy.logdebug("%s: >> brain found" % self.name)

        ##    Registrace zarizeni u AROM brain
        ##    > Arom returns 1 - OK, 0 - False
        ###**************************************************
        RegisterDriver = rospy.ServiceProxy('/arom/RegisterDriver', arom.srv.RegisterDriver)
        registred = RegisterDriver(name = self.name, sname = self.name, driver = self.arg['driver'], device = self.arg['type'], service = 'arom/driver/%s/%s' %(self.arg['type'], self.name), status = 1)
        rospy.loginfo("%s: >> register %s driver: %s" %(self.name, self.arg['driver'], registred))

        ##    Spusti se Action servis pro zmenu cile
        ###**************************************************
        #self.act = actionlib.SimpleActionServer('arom/%s/%s/target' %((self.arg['type'], self.name), arom.msg.MountTargetAction, execute_cb=self.ReciveTarget, auto_start = False)
        #self.act.start()

        ##    Ovladac se pripoji k montazi
        ###**************************************************
        if self.Autoconnect:
            self.connect()

        ##    Vytvoreni servisu na praci s montazi
        ###**************************************************
        self.driver = rospy.Service('/arom/driver/%s/%s' %(self.arg['type'], self.name), arom.srv.DriverControl, self.recieveHandler)

        ##    Publishera na jednoduche prikazy
        ###**************************************************
        #~~self.s_MountParameters =  rospy.Service('arom/mount/%s/parameter' %(self.name), arom.srv.MountParameter, self.MountParameter)
        self.pub_mount_position = rospy.Publisher('/arom/%s/%s/location' %(self.arg['type'], self.name), String, queue_size=10)


        ###    Vytvoreni subscribera na prijimani dat  - typ bude ==arom/mount/[jmeno]==
        ###**************************************************
        rospy.Subscriber('/arom/%s/%s' %(self.arg['type'],self.name), arom.msg.DriverControlm, self.recieveHandler)


        ###    Ovladac pujde ukoncit
        ###**************************************************

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            #self.update()
            slew_err = 0.01
            #if abs(self.mountParams['coordinates'].ra.degree - self.mountParams['coordinates']_target.ra.degree) > slew_err or abs(self.mountParams['coordinates'].dec.degree - self.mountParams['coordinates']_target.dec.degree) > slew_err:# and not self.slewing:
            if self.newTarget > 0:# and not self.slewing:
                #rospy.loginfo("Chyba namireni je %i a %i" %((self.mountParams['coordinates'].ra.degree - self.mountParams['coordinates']_target.ra.degree), abs(self.mountParams['coordinates'].dec.degree - self.mountParams['coordinates']_target.dec.degree)))
                #self.setPosition()
                pass
            self.getPosition()
            #self.getPosition("SkyCoord")
            #print self.mountParams['coordinates']
            self.pub_mount_position.publish("%f;%f;%s;%s;" %(self.mountParams['coordinates'].ra.degree, self.mountParams['coordinates'].dec.degree, self.mountParams['coordinates'].transform_to(AltAz(obstime = Time.now(), location=self.observatory['coordinates'])).alt.degree, self.mountParams['coordinates'].transform_to(AltAz(obstime = Time.now(), location=self.observatory['coordinates'])).az.degree))
            
            rate.sleep()
        
        self.stop()
        rospy.set_param('heq5/restoreloc', self.mountParams['coordinates'].transform_to(AltAz(obstime = Time.now(), location=self.observatory['coordinates'])).to_string('dms'))


    def recieveHandler(self, msg):
        if msg.type == 'function':
            try:
                if msg.data != '':
                    result = getattr(self, str(msg.name))(eval(msg.data))
                else:
                    result = getattr(self, str(msg.name))()
                return arom.srv.DriverControlResponse(data = repr(result), done = True)

            except Exception, e:
                rospy.logerr(e)
                return arom.srv.DriverControlResponse(data = repr(e), name = repr("Err"), done = False)

        #elif msg.type == 'Slew':
        #    self.mountParams['coordinates_target'] = SkyCoord(ra = float(data['ra']), dec = float(data['dec']), unit = 'deg')
        #    self.newTarget = 1
        #    rospy.logdebug("Souradnice nastaveny na: %s Aktualni jsou: %s Jejich rozdil je %s..." %(self.mountParams['coordinates_target'].to_string('dms') , self.mountParams['coordinates'].to_string('dms') ,self.mountParams['coordinates'].position_angle(self.mountParams['coordinates_target']).to_string() ))
        #    #return arom.srv.DriverControlRespond(done = True, data = "{'ra':"+str(self.mountParams['coordinates']_target.ra.degree)+"'ra':"+str(self.mountParams['coordinates']_target.dec.degree)+"}")
        elif msg.type == 'Stop':
            rospy.logerr("mount stop jeste neni implementovan")
            pass
        #elif msg.type == 'Sync':
        #    self.sync(SkyCoord(ra = float(data['ra']), dec = float(data['dec']), unit = 'deg'))
            
        elif msg.type == 'getPosition':
            rospy.logerr("mount sync jeste neni implementovan")
            return arom.srv.DriverControlResponse(name='nemo', type='type', data='data', done=True)
            #return arom.srv.DriverControlRespond(done = True, data = "{'ra':"+str(self.mountParams['coordinates'].ra.degree)+"'ra':"+str(self.mountParams['coordinates'].dec.degree)+"}")
        else:
            rospy.logerr("Neznama funkce")
            return arom.srv.DriverControlResponse(done = False, data = repr(None))
            

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

    def advSlew(self, param=None):
        param = eval(param)
        if param['type'] == 'radec':
            coord = SkyCoord(float(param['ra']), float(param['dec']), unit = 'deg', frame='icrs')
        elif param['type'] == 'altaz':
            coord = SkyCoord(alt=float(param['alt']), az=float(param['az']), frame='altaz', unit='deg', obstime = Time.now(), location = self.observatory['coordinates']).icrs

        self.mountParams['coordinates_target'] = coord
        self.newTarget += 1
        self.setPosition()
        return True
        
    def slew(self, param = None):
        raise NotImplementedError()

    def advSync(self, param = None):
        param = eval(param)
        if param['type'] == 'radec':
            coord = SkyCoord(float(param['ra']), float(param['dec']), unit = 'deg', frame='icrs')
        elif param['type'] == 'altaz':
            coord = SkyCoord(alt=float(param['alt']), az=float(param['az']), frame='altaz', unit='deg', obstime = Time.now(), location = self.observatory['coordinates']).icrs
        print "request on 'advSync", param, coord
        self.mountParams['coordinates'] = coord

    
    def sync(self, param = None):
        param = eval(param)
        self.mountParams['coordinates'] = SkyCoord(ra=param.ra, dec=param.dec, unit='degree', obstime=Time.now(), frame='icrs')
        #self.mountParams['coordinates_target'] = param

    def track(self, param = None):
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

    def setCoordinates(self, coordinates, unit = 'deg'):    # in degrees
        print self.mountParams['coordinates']
        self.mountParams['coordinates'] = SkyCoord(coordinates[0], coordinates[1], frame='icrs', unit=unit)
        return self.mountParams['coordinates']

    def setTarget(self, coordinates, unit = 'deg'):    # in degrees
        print self.mountParams['coordinates_target']
        self.mountParams['coordinates_target'] = SkyCoord(coordinates[0], coordinates[1], frame='icrs', unit=unit)
        return self.mountParams['coordinates_target']

    def getCoordinates(self, mode = 'radec'):    # in degrees
        if self.tracking:
            return self.mountParams['coordinates']
        else:
            return self.mountParams['coordinates'].altaz

    
    def _setHorizont(self, check = True):
        print self.arg['_horizont']
        self.horizont_check = check
        self.horizont = np.array([[0,0], [154,0], [155,45], [160,50], [170,20], [171, 0], [360,0]])
        self.horizont_hard = np.array([[0,-5], [154,-5], [155,40], [160,45], [170,15], [171,-5], [360,-5]])
        return self.horizont

    def LimitGuarder(self, loc = None):
        if loc == None:
            loc = self.mountParams['coordinates']

        altaz = loc.transform_to(AltAz(obstime = Time.now(), location=self.observatory['coordinates']))
        F_az = (np.abs(self.horizont[:,0] - altaz.az.degree)).argmin()
        local_horizont = self.horizont[F_az]

        F_az = (np.abs(self.horizont_hard[:,0] - altaz.az.degree)).argmin()
        local_horizont_hard = self.horizont_hard[F_az]

        rospy.logdebug("checking Limits: %s Pozice je: %s" %(repr(local_horizont), repr(altaz.to_string('dms'))))

        if altaz.alt.degree < local_horizont_hard[1]:
            rospy.logerr("TELESCOPE is in the forbidden area (azimuth: %i; height is %i, horizont: %i, house: %i)" %(altaz.az.degree, altaz.alt.degree, local_horizont[1], local_horizont_hard[1]))

        elif altaz.alt.degree < local_horizont[1]:
            rospy.logwarn("TELESCOPE is bellow horizont (azimuth: %i; height is %i, horizont: %i, house: %i)" %(altaz.az.degree, altaz.alt.degree, local_horizont[1], local_horizont_hard[1]))

        else:
            pass





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
        self.serialActive = False
        self.stepsPerRev={1:1, 2:1}
        self.data = ""  


        #=========================================
        #      EQmod serial comands
        #=========================================
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
        #=========================================
        #=========================================

        self.syncOffSet = [0,0]
        self.driverSyncTime = Time.now()
        self.coord_instrument_old = (0,0,Time.now())

        ##
        ##  Registrace ovladace jako mount
        ##

    def getPositionJSON(self):
        print "blablabla"
        JsonArr = []
        position = self.getPosition()
        print position
        return position


    def _axisPosToDeg(self, loc, ax=0):
        return loc / self.stepsPerRev[ax]/360.0

    def _axisPosToReal(self, loc, ax=0):
        return loc / self.stepsPerRev[ax]/360.0 + self.syncOffSet[ax]

    def _degToAxis(self, loc, ax=0):
        return loc * self.stepsPerRev[ax]/360.0

    def _degRealToAxis(self, loc, ax=0):
        return loc * self.stepsPerRev[ax]/360.0 + self.syncOffSet[ax]

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

        self._GetData(self.NotInstantAxisStop, self.Axis1,)
        self._GetData(self.NotInstantAxisStop, self.Axis2,)

    # Get mount name
        self.electronicVersion={}  ## X
        self.electronicVersion['raw']=self.Revu24str2long(self.mountParams['e']);
        self.electronicVersion['mountID'] = ((self.electronicVersion['raw'] & 0xFF) << 16) | ((self.electronicVersion['raw'] & 0xFF00)) | ((self.electronicVersion['raw'] & 0xFF0000) >> 16);
        self.mountID=self.electronicVersion['mountID'] & 0xFF;  ## X
        self.mountParams['MountModelID'] = self.electronicVersion['mountID'] & 0xFF

        if  self.electronicVersion ['mountID'] & 0xFF ==  0x00:
            self.electronicVersion['name'] = "EQ6"   ## X
            self.mountParams['MountModel'] = "EQ6"

        elif self.electronicVersion['mountID'] & 0xFF == 0x01:
            self.electronicVersion['name'] = "HEQ5"   ## X
            self.mountParams['MountModel'] = "HEQ5"

        elif self.electronicVersion['mountID'] & 0xFF == 0x02:
            self.electronicVersion['name'] = "EQ5"   ## X
            self.mountParams['MountModel'] = "EQ5"

        elif self.electronicVersion['mountID'] & 0xFF == 0x03:
            self.electronicVersion['name'] = "EQ3"   ## X
            self.mountParams['MountModel'] = "EQ3"

        elif self.electronicVersion['mountID'] & 0xFF == 0x80:
            self.electronicVersion['name'] = "GT"   ## X
            self.mountParams['MountModel'] = "GT"

        elif self.electronicVersion['mountID'] & 0xFF == 0x81:
            self.electronicVersion['name'] = "MF"   ## X
            self.mountParams['MountModel'] = "MF"

        elif self.electronicVersion['mountID'] & 0xFF == 0x82:
            self.electronicVersion['name'] = "114GT"   ## X
            self.mountParams['MountModel'] = "114GT"

        elif self.electronicVersion['mountID'] & 0xFF == 0x90:
            self.electronicVersion['name'] = "DOB"   ## X
            self.mountParams['MountModel'] = "DOB"

        elif self.electronicVersion['mountID'] & 0xFF == 0xF0:
            self.electronicVersion['name'] = "GEEHALEL"   ## X
            self.mountParams['MountModel'] = "GEEHALEL"
        else:
            self.electronicVersion['name'] = "Unknown_0x%x" %(self.electronicVersion['mountID'] & 0xFF)   ## X
            self.mountParams['MountModel'] = "Unknown_0x%x" %(self.electronicVersion['mountID'] & 0xFF)

            #ax_RA = self._GetData(self.GetAxisPosition, self.Axis1)
            #ax_DEC = self._GetData(self.GetAxisPosition, self.Axis2)


    # get Steps per one axcis revolution
        self.stepsPerRev={}   ## X
        self.stepsPerRev[self.AxRa] = self.Revu24str2long(self.mountParams['a1'])
        self.stepsPerRev[self.AxDec]= self.Revu24str2long(self.mountParams['a2'])
        self.mountParams['StepsPerRev'] = [self.stepsPerRev[self.AxRa], self.stepsPerRev[self.AxDec]]
        self.mountParams['StepsPerDeg'] = [self.stepsPerRev[self.AxRa]/360.0, self.stepsPerRev[self.AxDec]/360.0] # hodnoty pro HEQ5 jsou  [25066.666666666668, 25066.666666666668]

        rospy.loginfo("Steps per revolution: %s and per degree %s" %(repr(self.mountParams['StepsPerRev']), repr(self.mountParams['StepsPerDeg'])))
        

        print "getPosition('SkyCoord') >> ", self.getPosition("SkyCoord")
        self.getPosition()

        self.coord_instrument_old = (0,self.stepsPerRev[1]/2,Time.now())

        self.mountParams['tracking'] = True


    def slew(self, target = [None, None], unit = 'deg'):
        if target[0] != None and target[1] != None:
            self.mountParams['coordinates_target'] = SkyCoord(int(target[0]), int(target[1]), unit = unit)
            self.setPosition()

    def stop(self):
        self.tracking = False
        self._GetData(self.NotInstantAxisStop, self.Axis1,)
        self._GetData(self.NotInstantAxisStop, self.Axis2,)

    def _GetData(self, cmd, axis, param = None, max_time = 10):
        SkywatcherLeadingChar = ':'
        SkywatcherTrailingChar = '\r'

        if not param:
            req = "%c%c%c%c" %(SkywatcherLeadingChar, cmd, axis, SkywatcherTrailingChar)
        else:
            req = "%c%c%c%s%c" %(SkywatcherLeadingChar, cmd, axis, param, SkywatcherTrailingChar)

    ###    rospy.loginfo("Send data: '%s' >%s" %(str(cmd), repr(req)))
        while self.serialActive == True: time.sleep(0.05)
        self.serialActive = True
        self.ser.write(req)
        data = ""
        start_time = time.time()
        while 1:
            data = self.ser.read(999)
            if start_time + 5 < time.time():
                rospy.logerr("Serial port timout: 10s on message: '%s'" %(req))
                data = "!0\r"
                break
            if len(data) > 0:
                pass
            if ("#" in data) or ("=" in data) or ("!" in data):
                rospy.logdebug("Recived data:\t \t >>> %s" %(repr(data)))
                break
            time.sleep(0.05)
        self.serialActive = False

        if "=" in data:
            return data
        else:
            return data

    def ReciveTarget(self, target = None):
        rospy.loginfo("RecivedNewTarget %s" %(repr(target)))

    def setTimeMachine(self, on=None, mode=None):
        if on != None:
            self.tracking = on
        if mode != None:
            self.trackingMode = target
        update()

    def setPosition(self, param = None):
        try:
            self.getPosition()
            change = [0,0]
            change[0] = Angle(Angle(self.mountParams['coordinates_target'].ra) - Angle(self.mountParams['coordinates'].ra))
            change[1] = Angle(Angle(self.mountParams['coordinates_target'].dec) - Angle(self.mountParams['coordinates'].dec)).wrap_at('90d')
            #change[1] = Angle(0, unit='deg')
            print "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            rospy.loginfo( "Change: ]%s, %s] from: [%s] to: [%s]"%( str(change[0].degree),str(change[1].degree) , str(self.mountParams['coordinates'].to_string('dms')), str(self.mountParams['coordinates_target'].to_string('dms'))))

            ##
            ## Vyber smeru otaceni
            ##

            if change[0].degree < -180:
                dirRA = '21'
                change[0] = Angle(360 + change[0].degree, unit = 'deg')
            elif change[0] <= 0:
                dirRA = '20'
                change[0] = Angle(abs(change[0].degree), unit = 'deg')
            elif change[0].degree > 180:
                dirRA = '20'
                change[0] = Angle(360 - change[0].degree, unit = 'deg')
            elif change[0].degree > 0:
                dirRA = '21'
            else:
                dirRA = '21'
                self.logwarn("Problem in 'setPosition': Ra is out_of_range")


            if change[1].degree >= 0:
                dirDEC = '20'
            elif change[1].degree < 0:
                dirDEC = '21'


            stepsRA = self.mountParams['StepsPerDeg'][0] * abs(change[0].degree)
            stepsDEC = self.mountParams['StepsPerDeg'][1] * abs(change[1].degree)
            print ""
            print change, self.mountParams['StepsPerDeg'], "steps ra, dec", stepsRA, stepsDEC
            print ""

            rospy.loginfo("SLEW TO %s (%s) with difference %s" %(self.mountParams['coordinates'], self.getCoordinates, change))

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

            time.sleep(0.5)
            while True:
                ax0 = self._GetData(self.GetAxisStatus, self.Axis1)
                ax1 = self._GetData(self.GetAxisStatus, self.Axis2)
                if ax0[2] ==  "0" and ax1[2] ==  "0":
                    self.mountParams['coordinates'] = self.mountParams['coordinates_target']
                    self.time_data = time.time()
                    ra = self._GetData(self.NotInstantAxisStop, self.Axis2)
                    ra = self._GetData(self.SetMotionMode, self.Axis1, '20')  # reverse - '21'
                    self.newTarget -= 1
                    ra = self._GetData(self.StartMotion, self.Axis1)
                    break
                time.sleep(0.1)

            if self.mountParams['tracking'] == True:
                sp = self.long2Revu24str( int(self.mountParams['StepsPerRev'][0]/self.DayLenght[self.mountParams['trackingMode']])*6)
                ra = self._GetData(self.SetStepPeriod, self.Axis1, sp) 
                ra = self._GetData(self.StartMotion, self.Axis1)

        except Exception, e:
            rospy.logerr("Error in setCoordinates: %s" %(e))
            #print e
        print "coordinates, %s" %(str(self.mountParams['coordinates'].to_string('dms')))

        return self.mountParams['coordinates_target']

    def getPosition(self, Type = None):
        if Type == None:
            try:
                ax_RA = self._GetData(self.GetAxisPosition, self.Axis1)
                ax_DEC = self._GetData(self.GetAxisPosition, self.Axis2)

                ra = self.Revu24str2long(ax_RA)
                dec = self.Revu24str2long(ax_DEC)

                ra =  Angle((ra) / self.mountParams['StepsPerDeg'][0], unit='deg') - Angle( 380*60*60*24 / (Time.now() - self.driverSyncTime).sec, unit='deg')
                dec = Angle(0, unit='deg')
                
                self.LimitGuarder()

                

                return self.mountParams['coordinates']
            except Exception, e:
                rospy.logerr("Error in getPosition: %s" %(e))

        elif Type == 'SkyCoord':
            return self.mountParams['coordinates']

        elif str(Type) == 'JSON_ALL':
            radec = self.mountParams['coordinates']
            altaz = self.mountParams['coordinates'].transform_to(AltAz(obstime = Time.now(), location=self.observatory['coordinates']))
            return [radec.ra.degree, radec.dec.degree, altaz.alt.degree, altaz.az.degree]

        else: 
            rospy.logerr("neznamy typ (%s) v getPosition" %repr(Type))
            return -1



    #def getRealPosition(self, param = None):
    #    self.getPosition()
    ##    self.mountParams['coordinates']_real = (self.mountParams['coordinates'][0]+self.syncOffSet[0], self.mountParams['coordinates'][1]+self.syncOffSet[1])
    #    return self.mountParams['coordinates']_real

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

    def update(self, correction = False):
        old_ax1, old_ax2, old_time = self.coord_instrument_old
        coordinater = self.mountParams['coordinates']
        cas = Time.now()
        
        difference_time = cas - old_time

        if difference_time.sec > 30:
            rawRA = self._GetData(self.GetAxisPosition, self.Axis1)
            rawDEC= self._GetData(self.GetAxisPosition, self.Axis2)

            ax1 = self.Revu24str2long(rawRA)
            ax2 = self.Revu24str2long(rawDEC)

            difference_ax1 = ax1 - old_ax1
            difference_ax2 = ax2 - old_ax2

            rospy.loginfo("Update, ")

            predpokladRa = (self.stepsPerRev[0]/self.DayLenght[self.mountParams['trackingMode']]) * difference_time.sec

            rospy.logerr("rozdil predpokladu a aktualni pozici je %i" %(difference_ax1 - predpokladRa))

        if difference_time.sec > 30:
            self.coord_instrument_old = ax1, ax2, cas

        #self.mountParams['coordinates'].altaz = self.mountParams['coordinates'].transform_to(AltAz(obstime = Time.now(), location=self.observatory['coordinates']))

        #if abs(difference_ax1 - predpokladRa) > 100:
        #    print "> 100"

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
        self.mountParams['coordinates'] = [ra, dec]
        return self.mountParams['coordinates']

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
    cfg = rospy.get_param("ObservatoryConfig/file")
    with open(cfg) as data_file:
        config = json.load(data_file)
    for x in config:
        if x['name'] == sys.argv[1]:
            break
    mount = locals()[x['driver']](arg = x, name = x['name'], port = x['port'])
    #mount = EQmod()
