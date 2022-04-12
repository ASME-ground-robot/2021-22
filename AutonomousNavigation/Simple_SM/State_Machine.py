#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time

from rover.msg import sensors2smach

#find a way to import all files
from Subscriber import Subscribe_to
from Initialize_Rover import Initialize_Rover
from Waypoint_Navigation import Waypoint_Navigation

def code():
	print ("Starting State Machine...")
	time.sleep(1)
	main = smach.StateMachine(outcomes = ['Stop_Rover'])
	with main:
		smach.StateMachine.add('Initialize_Rover', Initialize_Rover(), transitions = {'Initialization_Completed':'Waypoint_Navigation', 'Initialization_Failed':'Stop_Rover'})

		smach.StateMachine.add('Waypoint_Navigation', Waypoint_Navigation(), transitions = {'Navigation_Completed':'Stop_Rover', 'Navigation_Failed':'Stop_Rover'})
		
	sis = smach_ros.IntrospectionServer('server' , main , '/tester')
	sis.start()
	outcome = main.execute()
	rospy.spin()
	sis.stop()

if __name__ == '__main__':
	rospy.init_node('sm')
	code()
