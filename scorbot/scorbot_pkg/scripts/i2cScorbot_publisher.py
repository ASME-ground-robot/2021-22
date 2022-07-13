#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from scorbot_pkg.msg import ScorbotPosition
#from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from math import pi
import xml.dom.minidom
import random
import smbus
import time
import struct
free_joints = {}
dependent_joints={}
joint_list = []
# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)
arrayToHome=[[0,1,0,900],[1,1,0,900],[2,1,0,900],[3,1,0,900],[4,1,0,900],[5,1,0,900]]

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
    estadoMotor= {'motor':motor,'state':rcvdVal[0],'position':posicionRcvd,'velocity':velocidadRcvd,'flag':flag}
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
        print ("error",motor)
        time.sleep(0.05)
    return countError

def MoveScorbot(arrayToMove):
    countError=0
    for i in range(0,6):
        print(arrayToMove[i])
        countError+=WriteMotorPos(arrayToMove[i][0],arrayToMove[i][1],arrayToMove[i][2],arrayToMove[i][3])
    return countError

def HomeScorbot():
    global doneHome
    countError=0
    if(int(motors['joint_1']['state']))!=35:
        print('joint_1',motors['joint_1']['state'])
        if(int(motors['joint_1']['state']))!=30:
            countError+=WriteMotorPos(0,3,0,0)
        else:
            pass
    else:
            if(int(motors['joint_2']['state']))!=35:
                print('joint_2',motors['joint_2']['state'])
                if(int(motors['joint_2']['state']))!=30:
##                    countError+=WriteMotorPos(0,1,0,900)
                    countError+=WriteMotorPos(1,3,0,0)
                else:
                    pass
            else:
                if(int(motors['joint_3']['state']))!=35:
                    print('joint_3',motors['joint_3']['state'])
                    if(int(motors['joint_3']['state']))!=30:
##                        countError+=WriteMotorPos(1,1,90,900)
                        countError+=WriteMotorPos(2,3,0,0)
                    else:
                        pass
                else:
                    if(int(motors['joint_4']['state']),int(motors['joint_5']['state']))!= (35,35):
                        print(motors['joint_4']['state'],motors['joint_5']['state'])
                        if(int(motors['joint_4']['state']),int(motors['joint_5']['state']))!= (30,30):
##                            countError+=WriteMotorPos(2,1,-90,900)
                            countError+=WriteMotorPos(3,3,0,0)
                            countError+=WriteMotorPos(4,3,0,0)
                        else:
                            pass
                    else:
##                        countError+=WriteMotorPos(3,1,90,900)
##                        countError+=WriteMotorPos(4,1,-90,900)
                        doneHome=1
                        print("Done")
    return countError

def enmergencyStop():
    countError=0
    #Ida al home del motor 1-3
    for i in range(0,6):
        countError+=WriteMotorPos(i,4,0,0)
    return countError

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value
def crearJoints():
    global free_joints
    global joint_list
    global dependent_joints
    global juntas
    global motors
    motors={}
    juntas={}
    description = get_param('robot_description')
    robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
    dependent_joints = get_param("dependent_joints", {})
    use_mimic = get_param('use_mimic_tags', True)
    use_small = get_param('use_smallest_joint_limits', True)
 # Find all non-fixed joints
    for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed' or jtype == 'floating':
                    continue
                name = child.getAttribute('name')
                joint_list.append(name)
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    try:
                        limit = child.getElementsByTagName('limit')[0]
                        minval = float(limit.getAttribute('lower'))
                        maxval = float(limit.getAttribute('upper'))
                    except:
                        rospy.logwarn("%s is not fixed, nor continuous, but limits are not specified!" % name)
                        continue

                safety_tags = child.getElementsByTagName('safety_controller')
                if use_small and len(safety_tags) == 1:
                    tag = safety_tags[0]
                    if tag.hasAttribute('soft_lower_limit'):
                        minval = max(minval, float(tag.getAttribute('soft_lower_limit')))
                    if tag.hasAttribute('soft_upper_limit'):
                        maxval = min(maxval, float(tag.getAttribute('soft_upper_limit')))

                mimic_tags = child.getElementsByTagName('mimic')
                if use_mimic and len(mimic_tags) == 1:
                    tag = mimic_tags[0]
                    entry = {'parent': tag.getAttribute('joint')}
                    if tag.hasAttribute('multiplier'):
                        entry['factor'] = float(tag.getAttribute('multiplier'))
                    if tag.hasAttribute('offset'):
                        entry['offset'] = float(tag.getAttribute('offset'))

                    dependent_joints[name] = entry
                    continue

                if name in dependent_joints:
                    continue

                joint = {'min': minval, 'max': maxval, 'zero': 0}
                joint['position'] = 0
                joint['velocity'] = 0.0
                joint['effort'] = 0.0
                if jtype == 'continuous':
                    joint['continuous'] = True
                free_joints[name] = joint
        
def actionScorbot(data):
    global stringOrder
    stringOrder=data.data

def enviarComandoMotores(data):
    global arrayToSend
    arrayToSend=[0,0,0,0,0,0]
    for  i in range(0,6):
        PosToSend_recieved=data.ScorbotAction[i]
        arrayToSend[i]=[PosToSend_recieved.motor,PosToSend_recieved.action,PosToSend_recieved.position,PosToSend_recieved.velocity]
##    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        
def doScorbot():
    global stringOrder
    if (stringOrder=="Home"):
        MoveScorbot(arrayToHome)
        stringOrder="Done"
    elif (stringOrder=="Move"):
        time.sleep(0.01)
        MoveScorbot(arrayToSend)
        stringOrder="Done"
    elif (stringOrder=="EnmStop"):
        stringOrder="Done"
        enmergencyStop()

def positionPublisher():
    crearJoints()
    count=0
    last_motors={}
    last_motors['joint_1']=readMotorValues(0)
    last_motors['joint_2']=readMotorValues(1)
    last_motors['joint_3']=readMotorValues(2)
    last_motors['joint_4']=readMotorValues(3)
    last_motors['joint_5']=readMotorValues(4)
    while not rospy.is_shutdown():
        #leer los datos de los arduinos
        dictReaded=readMotorValues(0)
        if(dictReaded['flag']==1):
            motors['joint_1']=last_motors['joint_1']
        else:
            motors['joint_1']=dictReaded
            last_motors['joint_1']=motors['joint_1']
        dictReaded=readMotorValues(1)
        if(dictReaded['flag']==1):
            motors['joint_2']=last_motors['joint_2']
        else:
            motors['joint_2']=dictReaded
            last_motors['joint_2']=motors['joint_2']
        dictReaded=readMotorValues(2)
        if(dictReaded['flag']==1):
            motors['joint_3']=last_motors['joint_3']
        else:
            motors['joint_3']=dictReaded
            last_motors['joint_3']=motors['joint_3']
        dictReaded=readMotorValues(3)
        if(dictReaded['flag']==1):
            motors['joint_4']=last_motors['joint_4']
        else:
            motors['joint_4']=dictReaded
            last_motors['joint_4']=motors['joint_4']
        dictReaded=readMotorValues(4)
        if(dictReaded['flag']==1):
            motors['joint_5']=last_motors['joint_5']
        else:
            motors['joint_5']=dictReaded
            last_motors['joint_5']=motors['joint_5']
       #publicar ScorbotPosition
        ms_msg=ScorbotPosition()
        i=0
        for k in motors:
            ms_msg.ScorbotAction[i].header.stamp = rospy.Time.now()
            ms_msg.ScorbotAction[i].header.frame_id = k
            ms_msg.ScorbotAction[i].motor=motors[k]['motor']
            ms_msg.ScorbotAction[i].action=motors[k]['state']
            ms_msg.ScorbotAction[i].position=motors[k]['position']
            ms_msg.ScorbotAction[i].velocity=motors[k]['velocity']
            i+=1
        juntas=motors
        juntas['joint_3']['position']*=-1
        juntas['joint_3']['position']-=(juntas['joint_2']['position'])
        juntas['joint_4']['position']*=-1
        dif4_5=juntas['joint_4']['position']-juntas['joint_5']['position']
        juntas['joint_4']['position']=(juntas['joint_4']['position']+juntas['joint_5']['position'])/2
        juntas['joint_4']['position']-=juntas['joint_2']['position']
        juntas['joint_4']['position']-=juntas['joint_3']['position']
        juntas['joint_5']['position']=dif4_5
        #publicar Jointstates
        js_msg = JointState()
        js_msg.header.stamp = rospy.Time.now()
        num_joints = (len(free_joints.items())+len(dependent_joints.items()))
        js_msg.position = num_joints * [0.0]
        js_msg.velocity = num_joints * [0.0]
        js_msg.effort = num_joints * [0.0]

        for i, name in enumerate(joint_list):
            js_msg.name.append(str(name))
            joint = None
           # Add Free Joint
            if name in free_joints:
                joint = free_joints[name]
                factor = 1
                offset = 0
           # Add Dependent Joint
            elif name in dependent_joints:
                param = dependent_joints[name]
                parent = param['parent']
                factor = param.get('factor', 1)
                offset = param.get('offset', 0)
                #Handle recursive mimic chain
                recursive_mimic_chain_joints = [name]
                while parent in dependent_joints:
                    if parent in recursive_mimic_chain_joints:
                        error_message = "Found an infinite recursive mimic chain"
                        rospy.logerr("%s: [%s, %s]", error_message, ', '.join(recursive_mimic_chain_joints), parent)
                        sys.exit(-1)
                    recursive_mimic_chain_joints.append(parent)
                    param = dependent_joints[parent]
                    parent = param['parent']
                    offset += factor * param.get('offset', 0)
                    factor *= param.get('factor', 1)
                joint = free_joints[parent]
           # hacer update con los datos que vienen del arduino
            joint['position']=juntas[name]['position']
            joint['velocity']=juntas[name]['velocity']
            joint['effort']=juntas[name]['state']
	   #joint['effort']=juntas[name]['effort']
            if 'position' in joint:
                js_msg.position[i] = joint['position'] *(pi/180) * factor + offset
            if  'velocity' in joint:
                js_msg.velocity[i] = joint['velocity'] *(pi/180) * factor
            if  'effort' in joint:
                js_msg.effort[i] = joint['effort']
        pub_motors.publish(ms_msg)
        pub_joints.publish(js_msg)
        doScorbot()
        count+=1
        rate.sleep()

rospy.Subscriber("comandToSend", ScorbotPosition, enviarComandoMotores)
rospy.Subscriber("StringToSend", String, actionScorbot)
pub_joints = rospy.Publisher('joint_states', JointState, queue_size=10)
pub_motors = rospy.Publisher('motor_states', ScorbotPosition, queue_size=10)
rospy.init_node('leyendoencoder', anonymous=True)
hz = get_param("rate", 20)
rate = rospy.Rate(hz) # 10hz
time.sleep(1)
stringOrder="waiting.."
doneHome=0
        
if __name__ == '__main__':
    try:
        positionPublisher()
    except rospy.ROSInterruptException:
        pass
