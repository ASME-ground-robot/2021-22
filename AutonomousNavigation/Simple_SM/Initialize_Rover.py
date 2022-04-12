#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time

from rover.msg import sensors2smach

from Subscriber import Subscribe_to

class Initialize_Rover(smach.State):
	def __init__(self):
		print("Initializing the Rover...")
		smach.State.__init__(self, outcomes = ['Initialization_Completed', 
						       'Initialization_Failed'])

            	#Subscribe to sensor nodes
		self.sensors_sub = Subscribe_to('sensors2smach')
		self.counter = 0
		time.sleep(2)

	def execute(self, sensor_data):
		self.counter = 0
		#Check if all sensors have published data
		sensors_data_sent = self.sensors_sub.was_data_sent()
		sensor_print = self.sensors_sub.get_data()
		print ("Sensor Data Sent:"),
		print (sensors_data_sent)
		print ("")
		print (sensor_print)

		# if the sensor data has not been received:
		while (sensors_data_sent == False):

			time.sleep(0.01)
			#continue checking if all sensors have published data
			sensors_data_sent = self.sensors_sub.was_data_sent()
			sensor_print = self.sensors_sub.get_data()
			print ("Sensor Data Sent:"),
			print (sensors_data_sent)
			print ("")
			print (sensor_print)
			print ("")

			#If any sensors have failed to publish after ~5 seconds, return failed
			if (self.counter > 500):
				return 'Initialization_Failed'
			self.counter = self.counter + 1

		#When all nodes are publishing data, return Finished
		return 'Initialization_Completed'

def code():
	rospy.init_node('sm')
	main = smach.StateMachine(outcomes = ['Finished', 'Not_Finished'])
	with main:
		smach.StateMachine.add('Initialize_Rover', Initialize_Rover(), transitions = {'Initialization_Completed':'Finished', 'Initialization_Failed':'Not_Finished'})

	sis = smach_ros.IntrospectionServer('server' , main , '/tester')
	sis.start()
	outcome = main.execute()
	sis.stop()
	rospy.spin()

if __name__ == '__main__':
	code()
