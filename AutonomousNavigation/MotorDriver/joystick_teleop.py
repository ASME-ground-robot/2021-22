#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16, Int16MultiArray, String
from sensor_msgs.msg import Joy

def startup():
    rospy.init_node('teleop_node', anonymous=False)
    rospy.Subscriber("joy", Joy, teleop, queue_size=1)
    rospy.spin()
    
def teleop(data):
    pub = rospy.Publisher('py_control', Int16MultiArray, queue_size = 1)

# for reference: b=[foward, 
#		    reverse, 
#		    point turn left,
#		    point turn right, 
#		    skid left, 
#		    skid right, 
#		    speed]

    b = [0,0,0,0,0,0,0]
    msg = Int16MultiArray()

    if data.axes[1] == 1:
        b = [1,0,0,0,0,0,1]
    if data.axes[1] == -1:
        b = [0,1,0,0,0,0,1]
    if data.axes[0] == 1:
        b = [0,0,1,0,0,0,1]
    if data.axes[0] == -1:
        b = [0,0,0,1,0,0,1]

    if data.buttons[4] == 1:
        b = [0,0,0,0,1,0,1]
    if data.buttons[5] == 1:
        b = [0,0,0,0,0,1,1]

    if data.axes[4] == 1:
        b[6] = 2
    if data.axes[4] == -1:
        b[6] = 4
    if data.axes[3] == -1:
        b[6] = 3

    msg.data = b
    pub.publish(msg)

if __name__ == '__main__':
    try:
        startup()
    except rospy.ROSInterruptException:
        pass
