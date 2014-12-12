import rospy, std_msgs.msg

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String

import owd_msgs.msg, owd_msgs.msg._WAMState, owd_msgs.msg._WAMInternals

import argparse, numpy, openravepy, time, IPython, math

# visualization
import matplotlib.pyplot as plt
import numpy as np
import copy
from scipy.spatial.distance import pdist

from HerbRobot import HerbRobot

class FancyMove():
	def __init__(self, planning_env, robot, style):
		self.planning_env = planning_env
		self.robot = robot

		self.magic_num = 5;
		self.n = 2;
		self.m = self.magic_num - self.n;
		self.n_joints = 7
		self.start = time.time()
		self.last_time_stamp = time.time();
		
		
		# lists 
		self.temp = [[] for i in xrange(self.n_joints)]
		self.DOF_values = [0 for i in xrange(self.n_joints)]
		self.old_DOF_values = [0 for i in xrange(self.n_joints)]
		self.joints = [[] for i in xrange(self.n_joints)]
		
		self.v0 = [0 for i in xrange(self.n_joints)]
		self.v0_old = [0 for i in xrange(self.n_joints)] # for calculating accel
		
		self.v1 = [0 for i in xrange(self.n_joints)]
		self.DOF_limit_low = [0 for i in xrange(self.n_joints)]
		self.DOF_limit_hig = [0 for i in xrange(self.n_joints)]

		self.accel = [[] for i in xrange(self.n_joints)]
		self.positions = [0 for i in xrange(self.n_joints)]
		self.dynamic_torque = [0 for i in xrange(self.n_joints)]
		self.dynamic_torque_old = [0 for i in xrange(self.n_joints)]

		# threshholds 
		self.thr = 0.02
		self.vel_limit = [0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6]
		self.num_stablize = 5
		self.thr_v0 = 0.1
		self.c_mean = 0
		self.noMoreMotionAllowed = False 
		self.actuation_count = 0
		self.freezeStart = 0
		self.max_speed = 0.4

		# gains
		self.epslon = 0.01
		self.small_gain = 0.9 
		self.small_angle = 0.2
		self.gains = 	   [10, 0, 0, 0, 0, 0, 0]
		self.torque_gain = [0.5, 0, 0, 0, 0, 0, 0]

		# costs
		self.total_cost = [0 for i in xrange(self.n_joints)]
		self.c_torque = [0 for i in xrange(self.n_joints)]
		self.c_DOF = [0 for i in xrange(self.n_joints)]
		self.c_t = [0 for i in xrange(self.n_joints)]
		self.c = [[] for i in xrange(self.n_joints)]
		self.mn0 = [0 for i in xrange(self.n_joints)]
		self.mn1 = [0 for i in xrange(self.n_joints)]
		self.c_t_list = [[] for i in xrange(self.n_joints)]

		# flag 
		self.pushed = True 

		# model
		self.models = []
		self.best_vars = []
		self.var_thr_weight = 1
		self.fs = [open('templates/sp', 'r'), open('templates/sr', 'r'), open('templates/sy', 'r'), open('templates/er', 'r'), open('templates/joint5', 'r'),open('templates/joint6', 'r'), open('templates/ep', 'r')]
		
		for f in self.fs:
			self.models.append(self.readModel(f))

		# touch sensor 
		self.action = "r"
		self.sensor_idx = 0
		self.touch_time = 0
		self.old_touch_time = 0

		# upper arm: 0 5 4 1
		#			 y w g r

		# lower arm: 2:inner pad 5:outter pad
		#			 b           w(b) 
		# 1: LSY 2: LSP 4: LE 
		self.mapping = {0:-1, 5:-2, 4:1, 1:2, 3:-4, 2:4}

		# write to file 
		self.count_write = 0
		self.f_c = open('data/c', 'w')
		self.f_mn0 = open('data/mn0', 'w')
		self.f_mn1 = open('data/mn1', 'w')
		self.f_v1 = open('data/v1', 'w')
		self.f_v0 = open('data/v0', 'w')

		self.style = style 
		self.desiredConfig = self.robot.left_arm.GetEndEffectorTransform()

		print "fancy move"

############################## Touch ########################################

	def getVelocityFromTouch(self):
		''' when there is a change from r to t, set the velocity to 0.2 '''
		touchSpeed = 0.5 

		if self.action == "t" and self.touch_time > self.old_touch_time:
			(jointToMove, sign) = self.mapTouchPadToJoint(self.sensor_idx)

			# simply change the sign of speed so it goes to opposite direction
			self.v1[jointToMove] = sign * touchSpeed 
			print 'touch'
			# print self.v1

		if self.action == "r" :
			(jointToMove, sign) = self.mapTouchPadToJoint(self.sensor_idx)
			
			self.v1[jointToMove] = 0.4*self.v0[jointToMove]

			if abs(self.v1[jointToMove]) < 0.1: self.v1[jointToMove] = np.sign(self.v1[jointToMove])*00.01
			print 'release'

		# reset touch time stamp 
		self.old_touch_time = self.touch_time


	def getVelocityFromTouch2(self):
		''' keep the end effector unmoved while moving one joint '''
	
		desiredEndEffectorMove = 0.5
		duration = 0.5
		a = 0.1
		desired_offset = [0,0,0]
		current_offset = (self.desiredConfig - self.robot.manip.GetEndEffectorTransform())[0:3,3]

		if self.action == "t" and self.touch_time > self.old_touch_time:

			# get desired offset from current config if touched 
			if self.sensor_idx == 0:
				desired_offset =  [0, 0, -a]
			if self.sensor_idx == 5:
				desired_offset = [-a, 0, 0] 
			if self.sensor_idx == 4:
				desired_offset = [0, 0, a] 
			if self.sensor_idx == 1:
				desired_offset = [a, 0, 0]  
			if self.sensor_idx == 3:
				desired_offset = [0, a, 0] 
			if self.sensor_idx == 2:
				desired_offset = [0, -a, 0] 
			# get desired config 
			self.desiredConfig[0:3,3] = self.robot.manip.GetEndEffectorTransform()[0:3,3] + desired_offset

		jacob = self.robot.manip.CalculateJacobian()
		self.v1 = np.dot(np.linalg.pinv(jacob), current_offset/duration)
		# print self.desiredConfig
		# print current_offset
		# print self.v1
			
		# if self.touch_time == self.old_touch_time:
		# 	self.v1[jointToMove] = 0.5 * self.v0[jointToMove] 

		# reset touch time stamp 
		# self.old_touch_time = self.touch_time


	def mapTouchPadToJoint(self, pad_idx):

		if pad_idx in self.mapping.keys():
			# -1 so that the indexing for joints are 0 1 3  
			return (abs(self.mapping[pad_idx]) - 1), np.sign(self.mapping[pad_idx]) 


################################ Push ########################################
	def isPushedTorqueIncrease(self, i):
    	# when cost goes high suddenly or low suddenly --> the arm is pushed, 
    	# i.e. the derivative of the cost is high in absoluate value 
    	
		if len(self.c[i]) > 20 and i == 0 and self.c_t[i]!=0: 
			self.c_mean = np.mean(self.c[i][-20:-3])
			# self.c_t_mean = np.mean(self.c_t[i][-20:-3])
			# print self.c_mean
			# print self.c[i][-2:]
			print "c_t"
			print self.c_t
			# if np.mean(self.c[i][-2:]) > 1.2*self.c_mean:
			# 	IPython.embed()

			if np.mean(self.c[i][-1:]) > 1.5*self.c_mean and self.c_t[i] > 0.5*self.getJointAverageFluctuation(i) and (not self.isActuating(i)) and self.c_t[i] > 0.5:	
				print "pushed torque increase "
				# IPython.embed()

				return True 
			else: return False 


	def isPushedTorqueDecrease(self, i):
		if len(self.c[i]) > 20 and i == 0 and self.c_t[i]!=0: 
			
			if np.mean(self.c[i][-1:]) < 0.5*self.c_mean and self.c_t[i] < -0.5*self.getJointAverageFluctuation(i) and (not self.isActuating(i)) and self.c_t[i] < -0.5:	
				print "pushed torque decrease"
				# IPython.embed()

				return True 
			else: return False 


	def atLowerLimitSide(self, i):
		''' see if joint current position is at lower or upper side'''

		if self.positions[i] - self.DOF_limit_low[i] < self.DOF_limit_hig[i] - self.positions[i]:
			return True
		else:
			return False 

	def isActuating(self, i):
		''' when the cost trajectory matches the template '''

		if self.dynamic_torque[i] > 1.2*self.dynamic_torque_old[i]:
			self.dynamic_torque_old[i] = self.dynamic_torque[i]
			print "is actuating"
			return True 
		else: return False 

    	
	def canPushFurther(self, i):
		''' when the robot is moving towards joint limit '''

		# cost from joint limit
		self.DOF_limit_low[i] = self.robot.GetDOFLimits()[0][i+11]
		self.DOF_limit_hig[i] = self.robot.GetDOFLimits()[1][i+11]

		if self.DOF_values[i] >= self.DOF_limit_hig[i] - self.small_angle: 
			return False 
		elif self.DOF_values[i] <= self.DOF_limit_low[i] + self.small_angle:
			return False 
		else:
			return True 


	def getVelocityFromPush(self):
		t_d = 0.5
	
		# calculate the new dof values 
		for i in xrange(1):

			# get new cost 
			# c[i] = self.getCost(i)
			self.c[i].append(self.getCost(i))

			self.DOF_values[i] += self.v1[i] * t_d
			# IPython.embed()

			# derivative of cost 
			if len(self.c[i]) >= 2:
			
				self.c_t[i] = self.c[i][-1] - self.c[i][-2]

				if self.c_t[i] > 1: 
					self.v1 = [0 for i in xrange(self.n_joints)]
					self.manip.SetStiffness(0)
					print " Warning!!! Going into gravity compensation mode"
					break 

				elif (time.time() - self.freezeStart) > 4:

					# proportional to the derivative of cost 
					if self.isPushedTorqueIncrease(i): 
						
						if self.atLowerLimitSide(i):
							# move to the lower end 
							print "increase -lowerend - move to lllllllllllllllllllllllllll"
							self.v1[i] = abs(self.gains[i]*self.c_t[i])
							
						else:
							print "incrase  -higherend -move to hhhhhhhhhhhhhhhhhhhhhhhhhhh "
							# move to the higher end 
							self.v1[i] = -abs(self.gains[i]*self.c_t[i])
					
					elif self.isPushedTorqueDecrease(i):

						if self.atLowerLimitSide(i):	
							print "decrease -lowerend move to hhhhhhhhhhhhhhhhhhhhhhhhhhh"
							self.v1[i] = -self.gains[i]*self.c_t[i]
						else:
							print "incrase  -higherend -move to llllllllllllllllllllllllll "
							# move to the lower end 
							self.v1[i] = self.gains[i]*self.c_t[i]
					else: 
						self.pushed = False 

				# print "v1"
				# print self.v1
				if abs(self.v1[i] - self.v0[i])> 0.2 and abs(self.v0[i] - self.v0_old[i]) > 0.2:
					print "nooooooooooooooooooooooooooooooooooooo"
					self.v1[i] = 0

				if self.isActuating(i):
					self.actuation_count += 1 
					self.v1[i] =  self.decay(self.v0[i])*self.v0[i]

					if self.actuation_count == 3:
						self.v1[i] =  self.decay(self.v0[i])*self.v0[i]
						print "freeze start"

						# start timer 
						self.freezeStart = time.time() 

						# reset 
						self.actuation_count = 0

				self.accel[i].append(self.v0[i] - self.v0_old[i]) 


				# go along with what you were doing before if nothing happens
				if self.v1[i] == 0:
					self.v1[i] = self.decay(self.v0[i])*self.v0[i]


				# TODO: check if v0_old the same as self.v0
				self.v0_old[i] = self.v0[i] 
				
			# IPython.embed()
			# print 'self.c_t'
			# print self.c_t
			# print "c"
			# print self.c
			# print self.v1
			# print self.v0
			
			# print 'mn1' 
			# print self.mn1
			# print 'mn0'
			# print self.mn0 
			

	def moveArm(self, msg):

		# call back 
		d = list(msg.data)

		for i in xrange(self.n_joints):
			self.temp[i] += [d[i]]
			if len(self.temp[0]) > self.magic_num:
				self.joints[i] = self.temp[i][-self.magic_num:]
				self.temp = [[] for i in xrange(self.n_joints)]

		# get the current dof values, and use it for forward collision check  
		self.DOF_values = self.robot.left_arm.GetDOFValues()

		# start from some number of fresh values 
		if len(self.joints) < self.num_stablize:
			return	
		else:

			# get new velocity from push then touch because touch is priority 
			# self.getVelocityFromPush()
			if self.style == '1':
				self.getVelocityFromTouch()  
			elif self.style == '2':
				self.getVelocityFromTouch2()  

			# decrease the speed when the arm is near joint limit 
			if not self.canPushFurther(i):
				print "reached joint limit!"
				self.v1[i] = 0.1*self.v0[i]

			# check velocity limits 
			if abs(self.v1[i]) > self.vel_limit[i]:
				self.v1[i] =  self.max_speed 


			time_clapsed = time.time() - self.start
			# print 'vars'
			# print self.best_vars
			# print 'v1'
			# print self.v1

			# self.writeToFile()

			if time_clapsed > 2: 
				# move the spercific DOF: 11 12 13 14 15 16 17
				# 0 shoulder, 1 shoulder, 2 forarm turn, 3 elbow up, 4 rotate wrist, 5 turn wrist, 6 rotate hand				
				# calculate the configuration to feed to PlanToConfig
				
				diff = np.linalg.norm(np.array(self.DOF_values - self.robot.left_arm.GetDOFValues()))

				# check goal collision
				robot_DOF_values = self.robot.GetDOFValues()
				# update LSY value 
				robot_DOF_values[11] = self.DOF_values[0]

				# don't go through the trouble of trying to move the arm 
				if np.linalg.norm(np.array(self.v1)) > self.thr:
					# print "inside"
					# only try it when it's likely to be executed 
					# if self.planning_env.isValid(robot_DOF_values)!=1:

					try:
						# print self.v1 
						# IPython.embed()
						self.robot.left_arm.Servo(self.v1)
					except:
						raise
					# else:
					# 	print "collision"

				# end of execution, update everything 	
				self.old_DOF_values = copy.deepcopy(self.DOF_values)
				self.pushed = True 

	def velocityCallBack(self, msg):
		self.v0 = msg.data
		if self.v0 < self.thr_v0 and self.v0 > -self.thr_v0:
			self.v0 = 0

	def touchSensorCallBack(self, msg):
		msg_list = msg.data.split(' ')
		self.action = msg_list[0]
		self.sensor_idx = int(msg_list[1])
		self.touch_time = msg_list[2]


		# IPython.embed()
		print self.action, self.sensor_idx, self.touch_time

	
	def positionCallBack(self, msg):
		self.positions = msg.positions
		# print 'position' 
		# print self.positions[0]
	
	def dynamicTorqueCallBack(self, msg):
		self.dynamic_torque = msg.data


	def Listener(self):
		
		rospy.init_node('filteredTorqueListener', anonymous=True)
		topic = rospy.get_param('~topic', 'torque_topic')
		rospy.Subscriber(topic, Float64MultiArray, self.moveArm)

		
		topic2 = rospy.get_param('~topic2', 'velocity_topic')
		rospy.Subscriber(topic2, Float64MultiArray, self.velocityCallBack)

		topic3 = rospy.get_param('~topic3', 'arduinoROSNode')
		rospy.Subscriber(topic3, String, self.touchSensorCallBack)

		# topic4 = rospy.get_param('~topic4', '/left/owd/waminternals')
		# rospy.Subscriber(topic4, owd_msgs.msg.WAMInternals, self.positionCallBack)

		# topic5 = rospy.get_param('~topic5', 'dynamic_torque_topic')
		# rospy.Subscriber(topic5, Float64MultiArray, self.dynamicTorqueCallBack)

		rospy.spin()


	def getCost(self, i):
		'''1. larger torque, larger cost
		   2. closer to joint limit, larger cost '''

		# cost from torque 
		self.mn1[i] = sum(self.joints[i][-self.m:])/self.m  
		# self.mn0[i] = sum(self.joints[i][-self.magic_num:-self.m])/(self.n) 

		# diff = self.mn1[i] - self.mn0[i] 

		self.c_torque[i] = abs(self.torque_gain[i] * self.mn1[i]) # self.c_torque can be + or -  

	    # total cost 
		self.total_cost[i] = self.c_torque[i] 

		return self.total_cost[i]

#################################### compare with template ########################################

	def getCostWithTemplate(self, i):

		# get the best match from template
		(best_var, best_data) = self.compare(self.models[i], self.joints[i])
		
		self.best_vars.append(best_var)
		# print "best var" + str(best_var)

		# set flag 
		if best_var < self.var_thr_weight: 
			self.is_actuating = True
			# update threshhold to be proportional to the best vars TODO 
			self.var_thr_weight = self.var_thr_weight*20/np.mean(np.array(self.best_vars))

		else: self.is_actuating = False 
 

		# call get cost 
		return self.getCost(i)


	def getJointAverageFluctuation(self, i):
		''' get the average fluctuation of the torque data '''

		j = self.joints[i] # 50 values
		return 0.5*np.std(j)


	def decay(self, value):
		''' return a new value that's smaller with the value and between 0 and 1 '''

		return 1.5*value*value

	def compare(self, model, data):
	    ''' find out the smallest distance with the model '''

	    l_m = len(model)
	    data = np.array(data)
	    model = np.array(model)
	    min_var = 500 
	    var = []
	    min_idx = 0
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
		# self.f.write(str(self.c_t))
		# self.f.write('\n')

		self.f_c.write(str(self.c))
		self.f_c.write('\n')

		self.f_mn1.write(str(self.mn1[0]))
		self.f_mn1.write('\n')

		self.f_mn0.write(str(self.mn0[0]))
		self.f_mn0.write('\n')

		self.f_v1.write(str(self.v1))
		self.f_v1.write('\n')

		self.f_v0.write(str(self.v0))
		self.f_v0.write('\n')

	    # self.f.write('diff ')
	    # self.f.write(str(self.DOF_values - self.robot.left_arm.GetDOFValues()))
	    # self.f.write('\n')
