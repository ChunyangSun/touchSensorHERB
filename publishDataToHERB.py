#!/usr/bin/env python
import rospy 

from std_msgs.msg import String
import serial


class arduinoROSNode: 
	"""this version is different from what's on herb0"""
	def __init__(self):

		print "hello ROS"
		self.initializeROSNode()


	def getSerialFromArduino(self):

		serialAddresses = list({'/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyS0','/dev/ttyS4', '/dev/COM1','/dev/COM2','/dev/COM3'})

		for serialAddress in serialAddresses:
			try:
				serial.Serial(serialAddress, 115200)
				if raw_input(serialAddress) == "y":
					break 
			except:
				print "exception"
				pass

		ser = serial.Serial(serialAddress, 115200)
		print ser 		
		while not rospy.is_shutdown():
			#rospy.loginfo(locationDict)
			# print 'rospy' 
			if (ser.inWaiting() > 0):
				try:
					duration = rospy.get_time() - self.start
					if ser.read() == 't':
						self.touched = ser.read()
				    	print "touched: " + str(self.touched)
				    	str_t = "t" + " " + self.touched + " " + str(duration) 
				    	self.pub.publish(str_t)
				    	ser.flushInput()

					if ser.read() == 'r':
						self.released = ser.read()
						print "released: " + str(self.released)
						str_r = "r" + " " + self.touched + " " + str(duration)
						self.pub.publish(str_r)
			
				except Exception, e:
				    print "ERROR: ", e
				    continue


	def initializeROSNode(self):
		print "hello ROS"

		self.pub = rospy.Publisher("arduino_touch_sensor_topic", String, queue_size = 10)
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

			# r.sleep()

if __name__ == '__main__':
	try:
		obj = arduinoROSNode()
		obj.getSerialFromArduino()

	except rospy.ROSInterruptException: pass
