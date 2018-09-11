#!/usr/bin/env python
# -*- coding: utf-8 -*-



#
# Protokol pro posilani a prijimani dat ze Stellaria
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
        
        TCP_IP = '0.0.0.0'
        TCP_PORT = rospy.get_param('port', 1234)
        BUFFER_SIZE = 1024

        self.dataRaDec = {'ra': 0, 'dec':0}
        self.dataSocket = ""
        self.NewPos = False

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print("Adresa {}:{}".format(TCP_IP, TCP_PORT))
        self.s.bind((TCP_IP, TCP_PORT))
        self.s.listen(1)
        print "cekam na spojeni"
        self.conn, addr = self.s.accept()
        self.conn.setblocking(False)
        print('Connected by', addr)
        #self.s.setblocking(0)
        self.s.settimeout(0.5)
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
            print('Incoming connection from %s' % repr(addr))
            handler = EchoHandler(sock)

    def  run(self):
        try:
            print("spostim smycku")
            rospy.Subscriber("/mount/status/coordinates/RaDec", Float32MultiArray, self.callback_RaDec)
            self.sendCoord = rospy.Publisher('/mount/controll', String, queue_size=5)
            rospy.init_node('AROM_stellarium')
            print("node inicializovan")
            data_rec = None

            while not rospy.is_shutdown():
                self.conn.send(self.SendPosition())
                try:
                    data_rec = self.conn.recv(1024)
                    if data_rec:
                        self.GetPosition(data_rec)
                        data_rec = None
                        print ("Diky")
                except Exception as e:
                    if e.args[0] == 11:
                        # nejsou nova data
                        pass
                    else:
                        print e, e.args[0]
                time.sleep(0.2)
            self.s.close()
                
        except Exception as e:
            self.s.close()
            print(e, e.args)
            self.__init__()


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
            return str(bytes(array))
            
        except Exception as e:
            print("ERROR1:", repr(e))

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
            if dec < 0xffffffff/2: dec = (dec*360.0/0xffffffff)
            else: dec = -((0xffffffff-dec)*360.0/0xffffffff)
            print ra, dec


            out = {
                'LENGTH': data[0] + data[1],
                'TYPE': data[2] + data [3],
                'TIME': time
            }
            self.sendCoord.publish("radec %f %f"%(ra, dec))
        except Exception as e:
            raise e
       



if __name__ == '__main__':
    StellariumRemote()
    asyncore.loop()
















