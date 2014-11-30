# listener and subscriber  
import rospy, std_msgs.msg
from std_msgs.msg import Float64MultiArray

import argparse, numpy, openravepy, time, IPython, math

# visualization
import matplotlib.pyplot as plt
import numpy as np

from HerbRobot import HerbRobot

class SimpleMove():
	def __init__(self, planning_env, robot):
		self.planning_env = planning_env
		self.robot = robot
		self.magic_num = 30;
		self.n = 20;
		self.m = self.magic_num - self.n;
		
		self.start = time.time()
		self.last_time_stamp = time.time();

		self.dof_values = []
		self.n_joints = 7
		
		# lists 
		self.old_dof_values = [0 for i in xrange(self.n_joints)]
		self.joints = [[] for i in xrange(self.n_joints)]
		self.v0 = [0 for i in xrange(self.n_joints)]
		self.v1 = [0 for i in xrange(self.n_joints)]
		self.DOF_limit_low = [0 for i in xrange(self.n_joints)]
		self.DOF_limit_hig = [0 for i in xrange(self.n_joints)]

		# threshholds 
		self.thr = 0.1
		self.der_thr = 0.1
		self.max_speed = 0.5
		self.vel_limit = [0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75]

		# gains
		self.epslon = 0.01
		self.small_gain = 0.9 
		self.max_cost = 5
		self.small_angle = 0.5
		self.gains = 	   [0.5, 0, 0, 0, 0, 0, 0]
		self.dof_gain =    [2, 2, 2, 2, 2, 2, 2]
		self.torque_gain = [2, 1, 1, 1, 1, 1, 1]



		# costs
		self.total_torque = [0 for i in xrange(self.n_joints)]
		self.c_torque = [0 for i in xrange(self.n_joints)]
		self.c_DOF = [0 for i in xrange(self.n_joints)]
		self.c_old = [0 for i in xrange(self.n_joints)]
		self.c_t = [0 for i in xrange(self.n_joints)]

		# write to file 
		self.count_write = 0
		self.f = open('torque_log_simple_move', 'w')

		print "simple move"


	def getCost(self, i):
		'''1. larger torque, larger cost
		   2. closer to joint limit, larger cost '''

		self.v0[i] = self.robot.manip.GetVelocity()[i]

		# cost from torque 
		self.mn1 = sum(self.joints[i][-self.m:])/self.m  
		self.mn0 = sum(self.joints[i][-self.magic_num:-self.m])/(self.n) 
		diff = self.mn1 - self.mn0 
		print "diff"
		print diff 

		self.c_torque[i] = abs(self.torque_gain[i] * diff) # self.c_torque can be + or -  

		# cost from joint limit
		self.DOF_limit_low[i] = self.robot.GetDOFLimits()[0][i+11]
		self.DOF_limit_hig[i] = self.robot.GetDOFLimits()[1][i+11]

		if self.DOF_values[i] >= self.DOF_limit_hig[i] - self.small_angle or self.DOF_values[i] <= self.DOF_limit_low[i] + self.small_angle:
			self.c_DOF[i] = self.max_cost 

		if self.DOF_values[i] - self.DOF_limit_low[i] > self.DOF_limit_hig[i] - self.DOF_values[i]:
			self.c_DOF[i] = self.dof_gain [i] * (self.DOF_limit_hig[i] - self.DOF_values[i])
		else:
			self.c_DOF[i] = self.dof_gain[i] * (self.DOF_values[i] - self.DOF_limit_low[i])

	    # total cost 
		self.total_torque[i] = self.c_torque[i] + self.c_DOF[i] 

		return self.total_torque[i]


	def calculateVelocity(self):
		t_d = time.time() - self.last_time_stamp

		# get the current dof values, and use it for forward collision check  
		self.DOF_values = self.robot.left_arm.GetDOFValues()
		c = [0 for i in xrange(self.n_joints)]

		# calculate the new dof values 
		for i in xrange(len(self.joints)-1):

			self.DOF_values[i] += self.v1[i] * t_d

			# get new cost 
			c[i] = self.getCost(i)

			# derivative of cost 
			self.c_t[i] = c[i] - self.c_old[i]

			# proportional to the derivative of cost 
			if self.c_t[i] < 0 and abs(self.c_t[i]) < self.der_thr: 
				self.v1[i] = - (1 - self.c_t[i]) * self.v0[i] 
			
			elif self.c_t[i] > 0 and abs(self.c_t[i]) < self.der_thr:
				self.v1[i] = (1 - self.c_t[i]) * self.v0[i]

			elif self.c_t[i] < 0 and abs(self.c_t[i]) >= self.der_thr:
				# arm get pushed opposite to gravity, send a negative command portional to c_t  
				self.v1[i] = self.v0[i] - self.gains[i] * abs(self.c_t[i])

			else: # arm get pushed along gravity, send a positive command porportional to c_t   
				self.v1[i] = self.v0[i] + self.gains[i]*abs(self.c_t[i])

			# check velocity limits
			if abs(self.v1[i]) > self.vel_limit[i]:
				self.v1[i] = self.v1[i]/np.linalg.norm(self.v1) * self.max_speed 

		print 'self.c_t'
		print self.c_t
		print "c"
		print c 
		self.c_old = c
		self.old_DOF_values = self.DOF_values
		# IPython.embed()


	def moveArm(self, msg):
		# call back 
		d = list(msg.data)

		for i in xrange(self.n_joints):
			self.joints[i] += [d[i]]

		# start from some number of freshvalues 
		if len(self.joints[0]) < self.magic_num:
			return
			
		# disgard the old values since they don't affect us anymore
		elif len(self.joints[0]) > 3*self.magic_num:
			for i in xrange(self.n_joints):
				self.joints[i] = self.joints[i][-self.magic_num:]

		else:
			# start to get v1 and wait for some time before moving arm  
			self.calculateVelocity()

			time_clapsed = time.time() - self.start

			# end program in some time 
			if time_clapsed < 500 and time_clapsed > 2:    
				# move the spercific DOF: 11 12 13 14 15 16 17
				# 0 shoulder, 1 shoulder, 2 forarm turn, 3 elbow up, 4 rotate wrist, 5 turn wrist, 6 rotate hand				
				# calculate the configuration to feed to PlanToConfig

				self.writeToFile()
				
				diff = np.linalg.norm(np.array(self.DOF_values - self.robot.left_arm.GetDOFValues()))

				print "v1"
				print self.v1

				# check goal collision
				robot_DOF_values = self.robot.GetDOFValues()
				robot_DOF_values[11] = self.DOF_values[0]


				# don't go through the trouble of trying to move the arm 
				if np.linalg.norm(np.array(self.v1)) > self.thr:

					# only try it when it's likely to be executed 
					if self.planning_env.isValid(robot_DOF_values)!=1:

						try:
							# IPython.embed()
							self.robot.left_arm.Servo(self.v1)

							# self.planAndExecute(self.DOF_values)

							# a = raw_input("enter") # debug
						except:
							raise
					else:
						print "collision"

				# self.visualize()
								
				self.last_time_stamp = time.time()


	def listenToFilteredTorque(self):
		
		rospy.init_node('filteredTorqueListener', anonymous=True)
		topic = rospy.get_param('~topic', 'fil_torque_topic')
		rospy.Subscriber(topic, Float64MultiArray, self.moveArm)

		rospy.spin()

	def planAndExecute(self, arm_dof):

		# rotation axis 
		self.robot.left_arm.Servo(self.v1)

		# @param target desired configuration
		# @param duration duration in seconds
		# @param timestep period of the control loop, in seconds
		# @param collisionchecking check collisions in the simulation environment
		# @return whether the servo was successful

		# self.robot.right_arm.ServoTo(arm_dof, 1, 0.01, True)


	def visualize(self):
		plt.figure(1)
		plt.subplot(711)
		plt.plot(self.joints[0], 'bo')

		plt.subplot(712)
		plt.plot(self.joints[1], 'bo')

		plt.subplot(713)
		plt.plot(self.joints[2], 'bo')

		plt.subplot(714)
		plt.plot(self.joints[3], 'bo')

		plt.subplot(715)
		plt.plot(self.joints[4], 'bo')

		plt.subplot(716)
		plt.plot(self.joints[5], 'bo')

		plt.subplot(717)
		plt.plot(self.joints[6], 'bo')

		plt.show()
		time.sleep(1)
		plt.close()


	def writeToFile(self):

	    self.count_write +=1 
	    # self.f.write('')
	    # self.f.write("joints ")
	    self.f.write(str(self.c_t))
	    self.f.write('\n')

	    # self.f.write('mn1 ')
	    # self.f.write(str(self.mn1))
	    # self.f.write('mn0 ')
	    # self.f.write(str(self.mn0))
	    # self.f.write('v1 ')
	    # self.f.write(str(self.v1))
	    # self.f.write('diff ')
	    # self.f.write(str(self.DOF_values - self.robot.left_arm.GetDOFValues()))
	    # self.f.write('\n')