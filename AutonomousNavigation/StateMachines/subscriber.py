import rospy
import threading
from rover.msg import cameras2smach
from rover.msg import controls2smach
from rover.msg import lidar2smach
from rover.msg import sensors2smach

topic_to_datatype = {'cameras2smach':cameras2smach, 'controls2smach':controls2smach,
		 'lidar2smach':lidar2smach, 'sensors2smach': sensors2smach}

class Subscribe_to():
	def __init__(self, topic):
		self.mutex = threading.Lock()
		self.foo = rospy.Subscriber(topic, topic_to_datatype[topic], self.callback)
		self.data = topic_to_datatype[topic]()
		self.data_sent = False

	def callback(self, cb_data): #Gets data from publisher
		self.mutex.acquire()
		self.data = cb_data
		self.data_sent = True
		self.mutex.release()

	def get_data(self): #Gives you most recent data collected
		self.mutex.acquire()
		self.final_data = self.data
		self.mutex.release()
		return self.final_data

	def was_data_sent(self): #Tells you if data was acquired yet
		self.mutex.acquire()
		self.check = self.data_sent
		self.mutex.release()
		return self.check
