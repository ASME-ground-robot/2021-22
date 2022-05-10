#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import math

from rover.msg import sensors2smach
from rover.msg import smach2controls

from Subscriber import Subscribe_to

class Waypoint_Navigation(smach.State):
	def __init__(self):
		print("Starting Waypoint Navigation...")
		smach.State.__init__(self, outcomes=['Navigation_Completed', 'Navigation_Failed'], input_keys=['goal'])

		self.sensors_sub = Subscribe_to('sensors2smach')
		self.setpoints_pub = rospy.Publisher('smach2controls', smach2controls, queue_size=1)
		self.setpoints = smach2controls()
		self.counter = 0
		time.sleep(2)

	def execute(self, userdata):
		sensors_data_sent = self.sensors_sub.was_data_sent()
		#print (sensors_data_sent)
		while (sensors_data_sent == False): #If no data received in 2 seconds, Failed
			time.sleep(0.01)
			sensors_data_sent = self.sensors_sub.was_data_sent()
			if (self.counter > 200):
				return 'Navigation_Failed'
			self.counter = self.counter + 1
		sensors_data = self.sensors_sub.get_data()

		goal_lat = 33.956436     # in pure degrees
		goal_lon = -118.269201  # in pure degrees

		print("Goal Latitude: "),
		print(goal_lat)
		print("Goal Longitude: "),
		print(goal_lon)

		current_lat = (sensors_data.current_latitude)
		current_lon = (sensors_data.current_longitude)

		Y_error = (goal_lat - current_lat)   #Y axis is Earth's N & S (unit of Lat degrees)
		X_error = (goal_lon - current_lon)   #X axis is Earth's E & W (unit of Lon degrees)

		# Aren's Helpful Comments:
		# One side of a field to another is about .001 degrees
		# One degree of Latitude remains mostly constant at 364,000 ft or ~110947.2 meters
		# One degree of Longitude depends on Latitude and average constant of ~111319.488 meters
		# USGS says one deg of Lon at 38 degrees Lat is 288,200 ft or ~87843.36 meters
		Y_error = (Y_error * 110947.2)
		X_error = (X_error * (111319.488 * (math.cos(math.radians(current_lat)))))

		# Calculating compass true North heading
		ang_from_Earths_X = math.degrees(math.atan(Y_error / X_error))

		# atan returns only between -90 to 90; this makes sure that the correct angles are returned
		if (X_error < 0):
			ang_from_Earths_X = ang_from_Earths_X + 180
		elif (Y_error < 0):
			ang_from_Earths_X = ang_from_Earths_X + 360 

		# Makes sure angle calculated is in the range 0-360 degrees
		if (ang_from_Earths_X < 0):
			ang_from_Earths_X = ang_from_Earths_X + 360

		# Change CCW angle to CW angle from True heading
		CW_ang_X = ((ang_from_Earths_X * (-1)) + 360)
		CW_ang_X = CW_ang_X + 90
		if (CW_ang_X < 0):
			CW_ang_X = CW_ang_X + 360
		if (CW_ang_X > 360):
			CW_ang_X = CW_ang_X - 360

		# Calculate angle error to turn to goal direction
		ang_Error = CW_ang_X - sensors_data.current_yaw
		
		# Sends the goal angle in range 0-360 degrees to the microcontroller
		self.setpoints.yaw_setpoint = sensors_data.current_yaw + ang_Error
		if (self.setpoints.yaw_setpoint > 360):
			self.setpoints.yaw_setpoint = self.setpoints.yaw_setpoint - 360
		elif (self.setpoints.yaw_setpoint < 0):
			self.setpoints.yaw_setpoint = self.setpoints.yaw_setpoint + 360
		print("Heading Angle: "),
		print(self.setpoints.yaw_setpoint)

		# Sends the distance setpoint between the two coordinates to the microcontroller
		self.setpoints.distance_setpoint = math.sqrt((Y_error ** 2) + (X_error ** 2))
		print("Distance: "),
		print(self.setpoints.distance_setpoint)
		self.setpoints_pub.publish(self.setpoints)
		time.sleep(1)
		return 'Navigation_Completed'

def code():
	rospy.init_node('sm')
	main = smach.StateMachine(outcomes=['Finished', 'Not_Finished'])
	main.userdata.lat_lon = []
	with main:
		smach.StateMachine.add('Waypoint_Navigation', Waypoint_Navigation(), transitions={
		'Navigation_Completed':'Finished', 'Navigation_Failed':'Not_Finished'}, remapping={'goal':'lat_lon'})

	sis = smach_ros.IntrospectionServer('server', main, '/tester')
	sis.start()
	outcome = main.execute()
	sis.stop()
	rospy.spin()

if __name__ == '__main__':
	code()
