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

    b = [0,0,0,0,0]
    msg = Int16MultiArray()

    if data.axes[1] == 1:
        b = [1,0,0,0,1]
    if data.axes[1] == -1:
        b = [0,1,0,0,1]
    if data.axes[0] == 1:
        b = [0,0,1,0,1]
    if data.axes[0] == -1:
        b = [0,0,0,1,1]
    if data.axes[3] == 1:
        b[4] = 2
    if data.axes[3] == -1:
        b[4] = 4
    if data.axes[2] == -1:
        b[4] = 3
    msg.data = b
    pub.publish(msg)
    #rate = rospy.Rate(10)
    #rate.sleep()

if __name__ == '__main__':
    try:
        startup()
    except rospy.ROSInterruptException:
        pass
