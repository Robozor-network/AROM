#!/usr/bin/env python
# -*- coding: utf-8 -*-



#
# http://yoestuve.es/blog/communications-between-python-and-stellarium-stellarium-telescope-protocol/
#

import select
import asyncore
import socket
import time
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import sys


import math
import re
import logging
from time import strftime, localtime
from string import replace


class EchoHandler(asyncore.dispatcher_with_send):
    def handle_read(self):
        data = self.recv(1024)
        if data:
            self.dataSocket = data
            #self.send(data)



class StellariumRemote(asyncore.dispatcher):
    def __init__(self):
        asyncore.dispatcher.__init__(self)
        
        TCP_IP = ''
        TCP_PORT = 1234
        BUFFER_SIZE = 1024

        self.dataRaDec = {'ra': 0, 'dec':0}
        self.dataSocket = ""
        self.NewPos = False

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((TCP_IP, TCP_PORT))
        self.s.listen(1)
        print "cekam na spojeni"
        self.conn, addr = self.s.accept()
        self.conn.setblocking(False)
        print('Connected by', addr)
        #self.s.setblocking(0)
        self.s.settimeout(1)
        #elf.clients = [self.s, conn]
        #print TCP_IP, ":", TCP_PORT
        self.run()


    def callback_RaDec(self, data):
        print data.data
        self.dataRaDec['ra']= data.data[0]
        self.dataRaDec['dec']= data.data[1]
        self.NewPos = True


    def handle_accept(self):
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            self.sock = sock
            print 'Incoming connection from %s' % repr(addr)
            handler = EchoHandler(sock)

    def  run(self):

        try:
            print "spostim smycku"
            rospy.Subscriber("/mount/status/coordinates/RaDec", Float32MultiArray, self.callback_RaDec)
            self.sendCoord = rospy.Publisher('/mount/controll', String, queue_size=5)
            rospy.init_node('AROM_stellarium')
            print "node inicializovan"
            data_rec = None

            while not rospy.is_shutdown():
                self.conn.send(self.SendPosition())
                try:
                    data_rec = self.conn.recv(1024)
                    if data_rec:
                        self.GetPosition(data_rec)
                        data_rec = None
                        print  "Diky"
                except Exception as e:
                    if e.args[0] == 11:
                        # nejsou nova data
                        pass
                    else:
                        print e, e.args[0]
                time.sleep(0.2)
            self.s.close()
                
        except Exception as e:
            print e, e.args

        finally:
            self.conn.close()



        '''

        try:         
            while not rospy.is_shutdown():
                if not dataSocket:
                    print "received data:", dataSocket
                    
                    try:
                        array = bytearray(dataSocket)
                        lenght = array[0] | array[1] << 8
                        type = array[2] | array[3] << 8
                        timed = array[4] | array[5]<< 8 | array[6]<< 8*2 | array[7] << 8*3 | array[8] << 8*4| array[9] << 8*5| array[10] << 8*6| array[11] << 8*7
                        ra  = array[12] | array[13]<< 8 | array[14]<< 8*2 | array[15] << 8*3
                        dec = array[16] | array[17]<< 8 | array[18]<< 8*2 | array[19] << 8*3

                        if dec > 0x40000000:
                            dec = dec-0xffffffff
                        if ra > 0xff000000:
                            ra = ra-0xffffffff

                        ra = ra/11930465
                        dec = dec/11930465

                        print "Slew - RaDec", ra, dec

                        #self.SendPosition()

                        
                    except Exception, e:
                        print e
                    dataSocket = None

                self.SendPosition()
                    
                
        except Exception, e:
            print e
        '''

    def SendPosition(self):
        try:
            #print "sendpos", dataRaDec
            lenght = 24
            typed = 0
            timed = int(time.time()*1000)
            status = 0
            ra = int(self.dataRaDec['ra']*11930465)
            dec = int(self.dataRaDec['dec']*11930465)
            status = 0
            
            #print "odesilam", lenght, typed, timed, status, ra, dec

            array = bytearray([ chr((lenght) & 0xFF), chr((lenght >> 8) & 0xFF),
                                chr((typed) & 0xFF), chr((typed >> 8) & 0xFF),
                                chr((timed) & 0xFF), chr((timed >> 8) & 0xFF), chr((timed >> 16) & 0xFF), chr((timed >> 24) & 0xFF), chr((timed >> 32) & 0xFF), chr((timed >> 40) & 0xFF), chr((timed >> 48) & 0xFF), chr((timed >> 64) & 0xFF),
                                chr((ra) & 0xFF), chr((ra >> 8) & 0xFF), chr((ra >> 16) & 0xFF), chr((ra >> 24) & 0xFF),
                                chr((dec) & 0xFF), chr((dec >> 8) & 0xFF), chr((dec >> 16) & 0xFF), chr((dec >> 24) & 0xFF),
                                chr((status) & 0xFF), chr((status >> 8) & 0xFF), chr((status >> 16) & 0xFF), chr((status >> 24) & 0xFF) ])
            #print "posilam", array
            return str(bytes(array))
            
        except Exception, e:
            print "ERROR1:", repr(e)

    def GetPosition(self, string):
        try:
            data = bytearray(string)
            print data, type(data)
            print bin(data[0]), bin(data[1])
            time = (data[4]<< 0 | data [5]<< 8 | data[6]<< 16 | data [7]<< 24 | data[8]<< 32 | data [9]<< 40 | data[10] << 48 | data [11] << 56)/1000.0

            ra = (data[12] << 0 | data[13] << 8 | data[14] << 16 | data[15] << 24)
            dec = (data[16] << 0 | data[17] << 8 | data[18] << 16 | data[19] << 24)

            print (hex(ra), hex(dec))
            if ra < 0xffffffff/2: ra = (ra*360.0/0xffffffff)
            else: ra = -((0xffffffff-ra)*360.0/0xffffffff)
            #ra = (ra*360.0/0xffffffff)
            if dec < 0xffffffff/2: dec = (dec*360.0/0xffffffff)
            else: dec = -((0xffffffff-dec)*360.0/0xffffffff)
            print ra, dec


            out = {
                'LENGTH': data[0] + data[1],
                'TYPE': data[2] + data [3],
                'TIME': time
            }
            self.sendCoord.publish("radec %f %f"%(ra, dec))
            #print data & 0xffff
            #print int(data & 0xffff)
        except Exception as e:
            raise e
       



if __name__ == '__main__':
    StellariumRemote()
    asyncore.loop()
















