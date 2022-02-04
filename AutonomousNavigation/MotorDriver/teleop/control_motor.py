#!/usr/bin/env python
import rospy
import roslib

from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String

ctrl = """
Reading commands from the keyboard! 

--------------------------------------
Moving around:

           (Foward)

               w    
 (Left)   a    s    d   (Right)

          (Backwards)
---------------------------------------

Note: To read key input, press key once and hit Enter before inputing another key.

Press CTRL-C then Enter to quit.
"""
print(ctrl)

def talker():
    rospy.init_node('control_motor', anonymous=False)

    pub = rospy.Publisher('py_control', Int16MultiArray, queue_size = 10)
    rate = rospy.Rate(10)
    
    b = [0,0,0,0,0]
    while not rospy.is_shutdown():
        msg = Int16MultiArray()
        i = raw_input()
        if (i == "w"):
            b = [1,0,0,0,1]
        elif (i == "s"):
            b = [0,1,0,0,1]
        elif (i == "a"):
            b = [0,0,1,0,1]
        elif (i == "d"):
            b = [0,0,0,1,1]
        elif (i == "p"):
            if b[4] <= 3:
                b[4] += 1
        elif (i == "o"):
            if b[4] >= 2:
                b[4] -= 1
        else:
            b = [0,0,0,0,0]
        msg.data = b
        pub.publish(msg)
	rate.sleep()


if __name__ == '__main__':
    talker()
