#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import math

#from rover.msg import sensors2smach
from rover.msg import smach2controls

#from Subscriber import Subscribe_to

class Waypoint_Navigation(smach.State):
	def __init__(self):
		print("Starting Waypoint Navigation...")
		smach.State.__init__(self, outcomes=['Navigation_Completed'])

		#self.sensors_sub = Subscribe_to('sensors2smach')
		self.setpoints_pub = rospy.Publisher('smach2controls', smach2controls, queue_size=1)
		self.setpoints = smach2controls()
		#self.counter = 0
		time.sleep(1)

	def execute(self, userdata):
		#sensors_data_sent = self.sensors_sub.was_data_sent()
		#print (sensors_data_sent)
		#while (sensors_data_sent == False): #If no data received in 2 seconds, Failed
		#	time.sleep(0.01)
		#	sensors_data_sent = self.sensors_sub.was_data_sent()
		#	if (self.counter > 200):
		#		return 'Navigation_Failed'
		#	self.counter = self.counter + 1
		#sensors_data = self.sensors_sub.get_data()

		self.setpoints.distance_setpoint = 5.00
		print(self.setpoints.distance_setpoint)
		self.setpoints.yaw_setpoint = 15.00
		print(self.setpoints.yaw_setpoint)
		self.setpoints_pub.publish(self.setpoints)
		time.sleep(1)
		return 'Navigation_Completed'

def code():
	rospy.init_node('sm')
	main = smach.StateMachine(outcomes=['Finished'])
	with main:
		smach.StateMachine.add('Waypoint_Navigation', Waypoint_Navigation(), transitions={
		'Navigation_Completed':'Finished'})

	sis = smach_ros.IntrospectionServer('server', main, '/tester')
	sis.start()
	outcome = main.execute()
	sis.stop()
	rospy.spin()

if __name__ == '__main__':
	code()
