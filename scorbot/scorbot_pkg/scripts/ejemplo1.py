#import cv2
import numpy as np
import gi
from math import pi
import random
import smbus
import time
import struct
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GLib, GdkPixbuf
#cap = cv2.VideoCapture(1)
bus = smbus.SMBus(1)
free_joints = {}
dependent_joints={}
joint_list = []

def readMotorValues(motor):
    rcvdVal=[0,0,0,0,0,0]
    adress=motor+4
    flag=0
    countError=0
    for i in range(0,6):
        try:
            rcvdVal[i]=(bus.read_byte(adress))
        except IOError:
            flag=1
            countError+=1
            #print "error",motor
            break
    if rcvdVal[5]!=adress+rcvdVal[0]:
        flag=1
        #print "error datos",motor
    posicionRcvd=(struct.unpack('<h',chr(rcvdVal[2])+chr(rcvdVal[1]))[0])/50
    velocidadRcvd=(struct.unpack('<h',chr(rcvdVal[4])+chr(rcvdVal[3]))[0])/50
    estadoMotor= {'motor':motor,'state':rcvdVal[0],'position':posicionRcvd*(pi/180),'velocity':velocidadRcvd*(pi/180),'flag':flag}
    return estadoMotor

def WriteMotorPos(motor,action,position,vel):
    cmd=1
    security=1
    adress=motor+4
    security=adress+action
    position1=position&0xff
    position2=(position>>8)&0xff
    vel1=vel&0xff
    vel2=(vel>>8)&0xff
    flag=1
    countError=0
    while (flag==1 and countError<5):
     flag=0
     try:
      bus.write_i2c_block_data(adress,cmd,[action,position1,position2,vel1,vel2,security])
     except IOError:
        flag=1
        countError+=1
        #print "error",motor
        time.sleep(0.05)
    return countError

def HomeScorbot():
    countError=0
    recibidoMotor=[0,0,0,0,0,0,0]
    #Ida al home del motor 4-5
    countError+=WriteMotorPos(3,3,0,0)
    countError+=WriteMotorPos(4,3,0,0)
    recibidoMotor[3]=readMotorValues(3)
    recibidoMotor[4]=readMotorValues(4)
    while (recibidoMotor[3]['state'],recibidoMotor[4]['state'] )!= (35,35):
        recibidoMotor[3]=readMotorValues(3)
        recibidoMotor[4]=readMotorValues(4)
    print ("home",recibidoMotor[3]['motor'],recibidoMotor[4]['motor'])
    #Ida al home del motor 1-3
    for i in range(0,3):
        countError+=WriteMotorPos(i,3,0,0)
        recibidoMotor[i]=readMotorValues(i)
        while recibidoMotor[i]['state'] != 35:
            recibidoMotor[i]=readMotorValues(i)
            time.sleep(0.05)
        print ("home",recibidoMotor[i]['motor'])
    #Ida al home del motor 6 (Gripper)
    countError+=WriteMotorPos(5,3,0,0)
    recibidoMotor[5]=readMotorValues(5)
    time.sleep(3)
    print ("home",recibidoMotor[5]['motor'])
    return countError

def enmergencyStop():
    countError=0
    for i in range(0,6):
        countError+=WriteMotorPos(i,4,0,0)
    return countError
inAction=0
def onDeleteWindow(self, *args):
        #cap.release()
        Gtk.main_quit()
def BttnClicked_EnmStop(self, *args):
        inAction=1
        enmergencyStop()
        inAction=0
def BttnClicked_Home(self, *args):
        inAction=1
        HomeScorbot()
        inAction=0 
def BttnClicked_Send(self, *args):
    speedToSend=speedBar.get_value()
    print (speedToSend)
def BttnClicked_BeginTransm(self, *args):
##        if(inAction==0):
##                try:
##                    positionPublusher()
##                except rospy.ROSInterruptException:
                    pass

builder = Gtk.Builder()
builder.add_from_file("ejemplo1.glade")
handlers={
    "onDeleteWindow":onDeleteWindow,
    "BttnClicked_EnmStop":BttnClicked_EnmStop,
    "BttnClicked_Home":BttnClicked_Home,
    "BttnClicked_Send":BttnClicked_Send,
    "BttnClicked_BeginTransm":BttnClicked_BeginTransm

}

window = builder.get_object("window1")
image = builder.get_object("image")
speedBar = builder.get_object("speedAdjustment")
window.show_all()
builder.connect_signals(handlers)

##def show_frame(*args):
##    ret, frame = cap.read()
##    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
##    frame = cv2.resize(frame, None, fx=0.7, fy=0.7, interpolation = cv2.INTER_CUBIC)
##    pb = GdkPixbuf.Pixbuf.new_from_data(frame.tostring(),
##                                        GdkPixbuf.Colorspace.RGB,
##                                        False,
##                                        8,
##                                        frame.shape[1],
##                                        frame.shape[0],
##                                        frame.shape[2]*frame.shape[1])
##    image.set_from_pixbuf(pb.copy())
##    return True
##
##GLib.idle_add(show_frame)
Gtk.main()
