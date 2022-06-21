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

//=======================================================================================================================

    b = [0,0,0,0,0]
    msg = Int16MultiArray()

    b = [data.axes[1],data.axes[0],data.axes[4],data.axes[3],1]
    
    if (data.axes[4] > 0.9)
    {
      b[4] = 2
    }
    if (data.axes[4] < -0.9)
    {
      b[4] = 4
    }
    if (data.axes[3] < -0.9)
    {
      b[4] = 3
    }
    if (data.axes[3] > 0.9)
    {
      b[4] = 1
    }
    
 //=============================================================================================================================================

    msg.data = b
    pub.publish(msg)

if __name__ == '__main__':
    try:
        startup()
    except rospy.ROSInterruptException:
        pass





