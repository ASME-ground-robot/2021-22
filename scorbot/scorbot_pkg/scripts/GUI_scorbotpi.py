#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity
import tf
import numpy as np
import gi
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState, CameraInfo, Image
from scorbot_pkg.msg import ScorbotPosition
from math import pi
import time
import struct
import cv2
from cv_bridge import CvBridge 
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GLib, GdkPixbuf
linksArray = ["/base_link","/link_1","/link_2","/link_3","/link_4","/link_5"]
arrayToSend={'motor':0,'action':0,'position':0,'velocity':0}
diccToSend={'joint_1':arrayToSend,'joint_2':arrayToSend,'joint_3':arrayToSend,'joint_4':arrayToSend,'joint_5':arrayToSend,'gripper':arrayToSend}
salir=0
#cap = cv2.VideoCapture(0)
def RadToGrad(x):
    return x*180/pi
    
def onDeleteWindow(self, *args):
    global salir
    #cap.release()
    salir=1
def BttnClicked_EnmStop(self, *args):
    strToSend = "EnmStop"
    string_pub.publish(strToSend)
def BttnClicked_Home(self, *args):
    sendBar_Base.set_value(0)
    sendBar_Hombro.set_value(0)
    sendBar_Codo.set_value(0)
    sendBar_MunecaP.set_value(0)
    sendBar_MunecaR.set_value(0)
    sendBar_Gripper.set_value(0)
    speedBar.set_value(0)
    strToSend = "Home"
    string_pub.publish(strToSend)
    
def BttnClicked_Send(self, *args):
    strToSend = "Move"
    diccToSend={'joint_1':arrayToSend,'joint_2':arrayToSend,'joint_3':arrayToSend,'joint_4':arrayToSend,'joint_5':arrayToSend,'gripper':arrayToSend}
    sendPos_Base=sendBar_Base.get_value()
    sendPos_Hombro=sendBar_Hombro.get_value()
    sendPos_Codo=-sendBar_Codo.get_value()
    sendPos_MunecaP=sendBar_MunecaP.get_value()
    sendPos_MunecaR=sendBar_MunecaR.get_value()*2
    sendPos_Muneca2=int(sendPos_MunecaP-(sendPos_MunecaR/2))
    sendPos_Muneca1=int(-(sendPos_MunecaR+sendPos_Muneca2))
    sendPos_Gripper=(sendBar_Gripper.get_value())
    speedToSend=speedBar.get_value()
    if(speedToSend==0):
        speedToSend=0
    else:
        speedToSend*=5.23
        speedToSend+=500
    diccToSend['joint_1'] = {'motor':0,'action':1,'position':sendPos_Base,'velocity':speedToSend}
    diccToSend['joint_2'] = {'motor':1,'action':1,'position':sendPos_Hombro,'velocity':speedToSend}
    diccToSend['joint_3'] = {'motor':2,'action':1,'position':sendPos_Codo,'velocity':speedToSend}
    diccToSend['joint_4'] = {'motor':3,'action':1,'position':sendPos_Muneca1,'velocity':speedToSend}
    diccToSend['joint_5'] = {'motor':4,'action':1,'position':sendPos_Muneca2,'velocity':speedToSend}
    diccToSend['gripper'] = {'motor':5,'action':1,'position':sendPos_Gripper,'velocity':speedToSend}
    comandsToSend=ScorbotPosition()
    i=0
    for k in diccToSend:
        comandsToSend.ScorbotAction[i].header.stamp = rospy.Time.now()
        comandsToSend.ScorbotAction[i].header.frame_id = k
        comandsToSend.ScorbotAction[i].motor=diccToSend[k]['motor']
        comandsToSend.ScorbotAction[i].action=diccToSend[k]['action']
        comandsToSend.ScorbotAction[i].position=diccToSend[k]['position']
        comandsToSend.ScorbotAction[i].velocity=diccToSend[k]['velocity']
        i+=1
    string_pub.publish(strToSend)
    jointCmd_pub.publish(comandsToSend)

def recibirMotorsState(data):
    global motorsState_Pos
    global motorsState_Vel    
    motorsState_Pos=[0,0,0,0,0,0]
    motorsState_Vel=[0,0,0,0,0,0]
    motorsState=ScorbotPosition()
    motorsState=data
    for i in range(0,5):
        motor=motorsState.ScorbotAction[i].motor
        motorsState_Pos[motor]=motorsState.ScorbotAction[i].position
        motorsState_Vel[motor]=motorsState.ScorbotAction[i].velocity
    
def printJointStates():
    rcvdPos_Base.set_text(str(motorsState_Pos[0]))
    rcvdPos_Hombro.set_text(str(motorsState_Pos[1]))
    rcvdPos_Codo.set_text(str(-motorsState_Pos[2]))
    rcvdPos_MunecaP.set_text(str(motorsState_Pos[3]))
    rcvdPos_MunecaR.set_text(str(motorsState_Pos[4]))
    #rcvdPos_Gripper.set_text(str(motorsState_Pos[5]))

    rcvdVel_Base.set_text(str(motorsState_Vel[0]))
    rcvdVel_Hombro.set_text(str(motorsState_Vel[1]))
    rcvdVel_Codo.set_text(str(motorsState_Vel[2]))
    rcvdVel_MunecaP.set_text(str(motorsState_Vel[3]))
    rcvdVel_MunecaR.set_text(str(motorsState_Vel[4]))
    #rcvdVel_Gripper.set_text(str(motorsState_Vel[5]))


def printGripperState():
    (trans,rot) = transformListener.lookupTransform(linksArray[combo_target_link.get_active()],linksArray[combo_source_link.get_active()], rospy.Time(0))
    RPY = tf.transformations.euler_from_quaternion(rot)
    RPY = list(map(RadToGrad,RPY))
    rcvdTrans_Gripper_x.set_text(str(format(trans[0],'.5f')))
    rcvdTrans_Gripper_y.set_text(str(format(trans[1],'.5f')))
    rcvdTrans_Gripper_z.set_text(str(format(trans[2],'.5f')))
    rcvdTrans_Gripper_roll.set_text(str(format(RPY[0],'.5f')))
    rcvdTrans_Gripper_pitch.set_text(str(format(RPY[1],'.5f')))
    rcvdTrans_Gripper_yaw.set_text(str(format(RPY[2],'.5f')))

def printUSBCamImage(image_msg):
    global image_data
    image_data=image_msg
    
builder = Gtk.Builder()
builder.add_from_file("/home/pi/scorbot_ws/src/scorbot_pkg/scripts/GUI_scorbotpi.glade")
handlers={
    "onDeleteWindow":onDeleteWindow,
    "BttnClicked_EnmStop":BttnClicked_EnmStop,
    "BttnClicked_Home":BttnClicked_Home,
    "BttnClicked_Send":BttnClicked_Send,

}
window = builder.get_object("window1")
image = builder.get_object("image")
sendBar_Base = builder.get_object("sendBar_Base")
sendBar_Hombro = builder.get_object("sendBar_Hombro")
sendBar_Codo = builder.get_object("sendBar_Codo")
sendBar_MunecaP = builder.get_object("sendBar_MunecaP")
sendBar_MunecaR = builder.get_object("sendBar_MunecaR")
sendBar_Gripper = builder.get_object("sendBar_Gripper")

speedBar = builder.get_object("speedAdjustment")

rcvdPos_Base = builder.get_object("rcvdPos_Base")
rcvdPos_Hombro = builder.get_object("rcvdPos_Hombro")
rcvdPos_Codo = builder.get_object("rcvdPos_Codo")
rcvdPos_MunecaP = builder.get_object("rcvdPos_MunecaP")
rcvdPos_MunecaR = builder.get_object("rcvdPos_MunecaR")
rcvdPos_Gripper = builder.get_object("rcvdPos_Gripper")

rcvdVel_Base = builder.get_object("rcvdVel_Base")
rcvdVel_Hombro = builder.get_object("rcvdVel_Hombro")
rcvdVel_Codo = builder.get_object("rcvdVel_Codo")
rcvdVel_MunecaP = builder.get_object("rcvdVel_MunecaP")
rcvdVel_MunecaR = builder.get_object("rcvdVel_MunecaR")
rcvdVel_Gripper = builder.get_object("rcvdVel_Gripper")

combo_target_link = builder.get_object("target_link")
combo_source_link = builder.get_object("source_link")
rcvdTrans_Gripper_x = builder.get_object("rcvdPos_Gripper_x")
rcvdTrans_Gripper_y = builder.get_object("rcvdPos_Gripper_y")
rcvdTrans_Gripper_z = builder.get_object("rcvdPos_Gripper_z")
rcvdTrans_Gripper_roll = builder.get_object("rcvdPos_Gripper_roll")
rcvdTrans_Gripper_pitch = builder.get_object("rcvdPos_Gripper_pitch")
rcvdTrans_Gripper_yaw = builder.get_object("rcvdPos_Gripper_yaw")

jointCmd_pub= rospy.Publisher('comandToSend', ScorbotPosition, queue_size=10)
string_pub= rospy.Publisher('StringToSend', String, queue_size=10)
rospy.Subscriber('driver/image_raw', Image,printUSBCamImage)
rospy.Subscriber('motor_states', ScorbotPosition,recibirMotorsState)
rospy.init_node('GUI_Scorbot', anonymous=True)
transformListener=tf.TransformListener()
time.sleep(2)
printJointStates()
window.show_all()
builder.connect_signals(handlers)


def show_frame(*args):
    bridge = CvBridge()
    imagen_array = bridge.imgmsg_to_cv2(image_data, "passthrough")
##    ret, frame = cap.read()
    frame = cv2.cvtColor(imagen_array, cv2.COLOR_BGR2RGB)
    frame = cv2.resize(frame, None, fx=0.8, fy=0.8, interpolation = cv2.INTER_CUBIC)
    frame=frame[::-1]
    arr=frame.tostring()
    pb = GdkPixbuf.Pixbuf.new_from_data(arr,
                                    GdkPixbuf.Colorspace.RGB,
                                    False,
                                    8,
                                    frame.shape[1],
                                    frame.shape[0],
                                    frame.shape[2]*frame.shape[1])
    image.set_from_pixbuf(pb.copy())
    return True

GLib.idle_add(show_frame)
if __name__ == '__main__':
    while not rospy.is_shutdown():
        printGripperState()
        printJointStates()
        if(Gtk.events_pending()):
            Gtk.main_iteration()
        if(salir):
            break
        
    
    
