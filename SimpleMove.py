# listener and subscriber  
import rospy, std_msgs.msg
from std_msgs.msg import Float64MultiArray

import argparse, numpy, openravepy, time, IPython

# visualization
import matplotlib.pyplot as plt
import numpy as np

from HerbRobot import HerbRobot

class SimpleMove():
	def __init__(self, planning_env, robot):
		self.planning_env = planning_env
		self.robot = robot
		
		self.n = 30;
		self.m = 20;
		
		self.start = time.time()
		self.last_time_stamp = time.time();

		self.dof_values = []
		self.n_joints = 7
		self.old_dof_values = [0 for i in xrange(self.n_joints)]
		self.joints = [[] for i in xrange(self.n_joints)]
		self.gains = [5, 0, 0, 0, 0, 0, 0] 
		self.thr = 0.1

		print "simple move"


	def calculateConfig(self):
		t_del = time.time() - self.last_time_stamp

		# get the current dof values 
		self.dof_values = self.robot.left_arm.GetDOFValues()

		# calculate the new dof values 
		for i in xrange(len(self.joints)-1):
				
			mn_new = sum(self.joints[i][-self.m:])/self.m  
			mn_base = sum(self.joints[i][-self.n:-self.m])/(self.n - self.m) 

			# get the velocity and make velocity proportional to torque 
			self.v0 = self.robot.manip.GetVelocity()[i]
			self.v1 = self.gains[i] * (mn_new - mn_base) + self.v0
			
			# get new DOF 
			self.dof_values[i] = self.dof_values[i] + self.v1 * t_del

			print "dof difference " 
			# print self.v1, t_del
			print self.dof_values - self.robot.left_arm.GetDOFValues()
		
		self.old_dof_values = self.dof_values
		


	def moveArm(self, msg):
		# call back 
		d = list(msg.data)

		for i in xrange(self.n_joints):
			self.joints[i] += [d[i]]

		if len(self.joints[i]) < 50:
			return
		else: 
			# make program end in some time 
			if time.time() - self.start < 500:    
				# move the spercific DOF: 11 12 13 14 15 16 17
				# 0 shoulder, 1 shoulder, 2 forarm turn, 3 elbow up, 4 rotate wrist, 5 turn wrist, 6 rotate hand				
				# calculate the configuration to feed to PlanToConfig
				self.calculateConfig()
				diff = np.linalg.norm(np.array(self.dof_values - self.robot.left_arm.GetDOFValues()))
				print 'diff'
				print diff 

				# check goal collision
				robot_dof_values = self.robot.GetDOFValues()
				robot_dof_values[11] = self.dof_values[0]

				# don't go through the trouble of trying to move the arm 
				if diff > self.thr:
					# only try it when it's likely to be executed 
					if self.planning_env.isValid(robot_dof_values)!=1:
						IPython.embed()
						try:
							print "inside try"
							# IPython.embed()
							# self.planAndExecute(self.dof_values)
							# dump feed to plantoconfig: slow
							self.robot.left_arm.PlanToConfiguration(self.dof_values)
							a = raw_input("enter") # debug

						except:
							raise
					else:
						print "collision"


				# self.visualize()
								
				self.last_time_stamp = time.time()
				
				# empty the joints 
				self.joints = [[] for i in xrange(self.n_joints)]

				# IPython.embed()


	def listenToFilteredTorque(self):
		
		rospy.init_node('filteredTorqueListener', anonymous=True)
		topic = rospy.get_param('~topic', 'fil_torque_topic')
		rospy.Subscriber(topic, Float64MultiArray, self.moveArm)
		rospy.spin()

	def planAndExecute(arm_dof):

		# interpolate and create plan 

		# convert to traj
		traj = self.convertPlanToTrajectory(plan)

		# execute traj
		self.executeTrajectory(traj)



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


