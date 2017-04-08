#!/usr/bin/env python
# -*- coding: utf-8 -*-

import select
import asyncore
import socket
import time
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import sys

dataRaDec = {'ra': 0, 'dec':0}
dataSocket = ""
NewPos = False

def callback_RaDec(data):
    global dataRaDec, NewPos
    print data.data
    dataRaDec['ra']= data.data[0]
    dataRaDec['dec']= data.data[1]
    NewPos = True

class EchoHandler(asyncore.dispatcher_with_send):
    def handle_read(self):
        global dataSocket
        data = self.recv(1024)
        if data:
            dataSocket = data
            #self.send(data)

class StellariumRemote(asyncore.dispatcher):
    def __init__(self):
        global NewPos, dataRaDec
        asyncore.dispatcher.__init__(self)
        
        TCP_IP = 'arom-weather.local'
        TCP_PORT = 1234
        BUFFER_SIZE = 1024

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((TCP_IP, TCP_PORT))
        self.s.listen(1)
        self.s.setblocking(0)
        self.s.settimeout(1)
        self.clients = [self.s]
        self.run()

    def handle_accept(self):
        pair = self.accept()
        if pair is not None:
            sock, addr = pair
            self.sock = sock
            print 'Incoming connection from %s' % repr(addr)
            handler = EchoHandler(sock)

    def  run(self):
        global NewPos, dataRaDec, dataSocket
        rospy.Subscriber("/mount/status/coordinates/RaDec", Float32MultiArray, callback_RaDec)

        rospy.init_node('AROM_stellarium')

        while not rospy.is_shutdown():
            inputReady, outputReady, exceptReady = select.select(self.clients, [], [])
            for x in inputReady: 
                print "ir", inputReady, x
                if x == self.s: 
                    csock, addr = self.s.accept() 
                    self.clients.append(csock) 

                else:
                    '''
                    if len(inputReady) > 1:
                        try:
                            print "============================="
                            data = x.recv(1024)
                            if data: 
                                print data 
                        except socket.timeout:
                            print "tieuout"
                    
                    #time.sleep(0.5)
                    '''
                    if True:
                        for i in self.clients:
                            if i is not self.s:
                                i.send(self.SendPosition())
                        NewPos = False
                    else: 
                        x.close() 
                        self.clients.remove(x)
                time.sleep(0.5)

            for i in self.clients:
                if i is not self.s:
                    i.send(self.SendPosition())
            NewPos = False


        self.s.close()


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
            global dataRaDec, NewPos
            #print "sendpos", dataRaDec
            lenght = 24
            typed = 0
            timed = int(time.time()*1000000)
            status = 0
            ra = int(dataRaDec['ra']*11930465)
            dec = int(dataRaDec['dec']*11930465)
            status = 0
            
            #print "odesilam", lenght, typed, timed, status, ra, dec

            array = bytearray([ chr((lenght) & 0xFF), chr((lenght >> 8) & 0xFF),
                                chr((typed) & 0xFF), chr((typed >> 8) & 0xFF),
                                chr((timed) & 0xFF), chr((timed >> 8) & 0xFF), chr((timed >> 16) & 0xFF), chr((timed >> 24) & 0xFF), chr((timed >> 32) & 0xFF), chr((timed >> 40) & 0xFF), chr((timed >> 48) & 0xFF), chr((timed >> 64) & 0xFF),
                                chr((ra) & 0xFF), chr((ra >> 8) & 0xFF), chr((ra >> 16) & 0xFF), chr((ra >> 24) & 0xFF),
                                chr((dec) & 0xFF), chr((dec >> 8) & 0xFF), chr((dec >> 16) & 0xFF), chr((dec >> 24) & 0xFF),
                                chr((status) & 0xFF), chr((status >> 8) & 0xFF), chr((status >> 16) & 0xFF), chr((status >> 24) & 0xFF) ])
            print "posilam", str(bytes(array))
            return str(bytes(array))
            
        except Exception, e:
            print "ERROR1:", repr(e)


if __name__ == '__main__':
    StellariumRemote()
    asyncore.loop()
