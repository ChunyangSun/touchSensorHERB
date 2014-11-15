#!/usr/bin/env python
import rospy 

from std_msgs.msg import String
import serial

class arduinoROSNode: 
	def __init__(self):

		print "hello ROS"

		self.pub = rospy.Publisher('topicTouch', String)
		rospy.init_node('arduinoPub', anonymous = True)
		self.r = rospy.Rate(10) # 10hz

	def getSerialFromArduino(self):

		serialAddresses = list({'/dev/ttyACM2', '/dev/ttyACM1', '/dev/ttyACM0', '/dev/ttyS0','/dev/ttyS4', '/dev/COM1','/dev/COM2','/dev/COM3'})

		for serialAddress in serialAddresses:
			try:
				serial.Serial(serialAddress, 115200)
				break
			except:
				print "exception!!!!"
				pass

		ser = serial.Serial(serialAddress, 115200)
		
		while not rospy.is_shutdown():
			#rospy.loginfo(locationDict)

			if (ser.inWaiting() > 0):
				try:
				    if ser.read() == 't':
				    	self.touched = ser.read()
				    	print "touched: " + str(self.touched)
				    	str_t = "t" + " " + self.touched + " " + str(rospy.get_time()) 
				    	self.pub.publish(str_t)
				    	ser.flushInput()
				    if ser.read() == 'r':
				    	self.released = ser.read()
				    	print "released: " + str(self.released)
				    	str_r = "r" + " " + self.touched + " " + str(rospy.get_time())
				    	self.pub.publish(str_r)
			
				except Exception, e:
				    print "ERROR: ", e
				    continue


	def initializeROSNode(self):
		print "hello ROS"

		pub = rospy.Publisher("arduinoROSNode", String, queue_size = 10)
		rospy.init_node('arduinoPub', anonymous = True)
		r = rospy.Rate(10) # 10hz

	def publishLocationData(self):
		
		while not rospy.is_shutdown():
			rospy.loginfo(locationDict)
			if self.touched !=-1:
				str_t = self.touched + "%s"%rospy.get_time()
				pub.publish(str)
			if self.released != -1:
				str_r = self.touched + "%s"%rospy.get_time()
				pub.publish(str)

			r.sleep()

if __name__ == '__main__':
	try:
		obj = arduinoROSNode()
		obj.getSerialFromArduino()

	except rospy.ROSInterruptException: pass
