# listener and subscriber  
import rospy, std_msgs.msg
from std_msgs.msg import Float64MultiArray

import argparse, numpy, openravepy, time, IPython, math

from scipy.spatial.distance import pdist
# visualization
import matplotlib.pyplot as plt
import numpy as np

from HerbRobot import HerbRobot

class SimpleMove():
	def __init__(self, planning_env, robot):
		self.planning_env = planning_env
		self.robot = robot
		self.magic_num = 40;
		self.n = 15;
		self.m = self.magic_num - self.n;
		
		self.start = time.time()
		self.last_time_stamp = time.time();

		self.dof_values = []
		self.n_joints = 7
		
		# lists 
		self.temp = [[] for i in xrange(self.n_joints)]
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
		self.small_angle = 0.2
		self.gains = 	   [0.5, 0, 0, 0, 0, 0, 0]
		self.dof_gain =    [1, 1, 1, 1, 1, 1, 1]
		self.torque_gain = [10, 1, 1, 1, 1, 1, 1]

		# costs
		self.total_cost = [0 for i in xrange(self.n_joints)]
		self.c_torque = [0 for i in xrange(self.n_joints)]
		self.c_DOF = [0 for i in xrange(self.n_joints)]
		self.c_old = [0 for i in xrange(self.n_joints)]
		self.c_t = [0 for i in xrange(self.n_joints)]
		self.DOF_too_low = [False for i in xrange(self.n_joints)]
		self.DOF_too_high = [False for i in xrange(self.n_joints)]

		# model
		self.models = []
		self.best_vars = []
		self.var_thr_weight = 0.1
		self.is_actuating = False 
		self.fs = [open('sp', 'r'), open('sp', 'r'), open('sp', 'r'), open('sp', 'r'), open('sp', 'r'),open('sp', 'r'), open('sp', 'r')]
		
		for f in self.fs:
			self.models.append(self.readModel(f))

		# write to file 
		self.count_write = 0
		self.f = open('torque_log_simple_move', 'w')

		print "simple move"


	def calculateVelocity(self):
		t_d = time.time() - self.last_time_stamp

		# get the current dof values, and use it for forward collision check  
		self.DOF_values = self.robot.left_arm.GetDOFValues()
		c = [0 for i in xrange(self.n_joints)]

		# calculate the new dof values 
		for i in xrange(len(self.joints)-1):

			# get new cost 
			# c[i] = self.getCost(i)
			c[i] = self.getCostWithTemplate(i)

			self.DOF_values[i] += self.v1[i] * t_d

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
		print 'move arm 1'
		# call back 
		d = list(msg.data)

		for i in xrange(self.n_joints):
			self.temp[i] += [d[i]]
			if len(self.temp[0]) > self.magic_num:
				self.joints = self.temp

		# start from some number of fresh values 
		if len(self.joints[0]) < self.magic_num:
			return
			
		# disgard the old values since they don't affect us anymore
		# elif len(self.joints[0]) > 2*self.magic_num:
		# 	for i in xrange(self.n_joints):
		# 		self.joints[i] = self.joints[i][-self.magic_num:]

		else:
			print 'move arm 2'
			# start to get v1 and wait for some time before moving arm  
			self.calculateVelocity()

			time_clapsed = time.time() - self.start
			# print 'vars'
			# print self.best_vars
			print 'v1'
			print self.v1

			# end program in some time 
			print self.DOF_too_high
			print self.DOF_too_low
			print self.is_actuating
			if time_clapsed > 2 and self.is_actuating == False and self.DOF_too_high == False and self.DOF_too_low == False: 
				# move the spercific DOF: 11 12 13 14 15 16 17
				# 0 shoulder, 1 shoulder, 2 forarm turn, 3 elbow up, 4 rotate wrist, 5 turn wrist, 6 rotate hand				
				# calculate the configuration to feed to PlanToConfig

				# self.writeToFile()
				print "inside"
				
				diff = np.linalg.norm(np.array(self.DOF_values - self.robot.left_arm.GetDOFValues()))

				# check goal collision
				robot_DOF_values = self.robot.GetDOFValues()
				robot_DOF_values[11] = self.DOF_values[0]

				# don't go through the trouble of trying to move the arm 
				if np.linalg.norm(np.array(self.v1)) > self.thr:

					# only try it when it's likely to be executed 
					if self.planning_env.isValid(robot_DOF_values)!=1:

						try:
							print 'try'
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


	def getCost(self, i):
		'''1. larger torque, larger cost
		   2. closer to joint limit, larger cost '''

		self.v0[i] = self.robot.manip.GetVelocity()[i]

		# cost from torque 
		self.mn1 = sum(self.joints[i][-self.m:])/self.m  
		self.mn0 = sum(self.joints[i][-self.magic_num:-self.m])/(self.n) 
		diff = self.mn1 - self.mn0 

		self.c_torque[i] = abs(self.torque_gain[i] * diff) # self.c_torque can be + or -  

		# cost from joint limit
		self.DOF_limit_low[i] = self.robot.GetDOFLimits()[0][i+11]
		self.DOF_limit_hig[i] = self.robot.GetDOFLimits()[1][i+11]

		if self.DOF_values[i] >= self.DOF_limit_hig[i] - self.small_angle: 
			self.DOF_too_high[i] = True 
		elif self.DOF_values[i] <= self.DOF_limit_low[i] + self.small_angle:
			self.DOF_too_low[i] = True
		else:
			self.DOF_too_low[i] = False 
			self.DOF_too_high[i] = False 


	    # total cost 
		self.total_cost[i] = self.c_torque[i] 

		return self.total_cost[i]

#################################### compare with template ########################################

	def getCostWithTemplate(self, i):

		# get the best match from template
		(best_var, best_data) = self.compare(self.models[i], self.joints[i])
		
		self.best_vars.append(best_var)

		# set flag 
		if best_var < self.var_thr_weight: 
			self.is_actuating = True
			# update threshhold to be proportional to the best vars TODO 
			self.var_thr_weight = self.var_thr_weight*20/np.mean(np.array(self.best_vars))
 
		else: 
			self.is_actuating = False 
		

		if self.is_actuating == True: print "actuating " 
		
		# call get cost 
		return self.getCost(i)



	def compare(self, model, data):
	    ''' find out the smallest distance with the model '''

	    l_m = len(model)
	    data = np.array(data)
	    model = np.array(model)
	    min_var = 500 
	    var = []
	    mn_m = [np.mean(model) in xrange(l_m)]

	    for i in xrange(len(data)- len(model)):

	        d = data[i: i+l_m]   
	        
	        mn_d = [np.mean(d) in xrange(l_m)]

	        var.append(pdist([model-mn_m, d - mn_d], 'euclidean'))
	        
	        # print vazip(model-mn_m, d - mn_d)r 
	        if min_var > var[i]:
	            min_var = var[i]
	            min_idx = i # TODO referenced before assignemtn 

	    # print var[3521:3530]
	    # print max(var)
	    # print min_idx
	    # IPython.embed()
	    # visualize1(data[min_idx : min_idx + l_m])
	    # visualize1(var)
	    return min_var, data[min_idx : min_idx + l_m]

	def readModel(self, f):
	    for line in f:
	    	model = line[1:-2].split(', ')
	   
	    for i in xrange(len(model)):
	        model[i] = float(model[i])

	    return model 

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
	def planAndExecute(self, arm_dof):

		# rotation axis 
		self.robot.left_arm.Servo(self.v1)

		# @param target desired configuration
		# @param duration duration in seconds
		# @param timestep period of the control loop, in seconds
		# @param collisionchecking check collisions in the simulation environment
		# @return whether the servo was successful

		# self.robot.right_arm.ServoTo(arm_dof, 1, 0.01, True)