#!/usr/bin/env python

# listener and subscriber  
import rospy, std_msgs.msg
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String 

# herbpy
import roslib
roslib.load_manifest("touchSensor")
import owd_msgs.msg, owd_msgs.msg._WAMState, owd_msgs.msg._WAMInternals

# visualization
import matplotlib.pyplot as plt

# others 
import time
import IPython
import numpy as np 
from numpy.linalg import inv 

class ListenToWAMInternals():
    def __init__(self, planner= None):
        self.planner = planner 
        # self.f = open('wamstate', 'r+')
        self.start = time.time()
        self.thr = 1
        self.iterval = 8
        self.n = 2
        self.counter = 0
        
        self.sp = []
        self.sr = []
        self.sy = []
        self.er = []
        self.joint5 = []
        self.joint6 = []
        self.ep =[]

        self.sp_fil = []
        self.sr_fil = []
        self.sy_fil = []
        self.er_fil = []
        self.joint5_fil = []
        self.joint6_fil = []
        self.ep_fil = []
        
        # factors for low pass filter
        self.factors = [0.3, 0.2, 0.1, 0.2, 0.2, 0.1, 0.2]
        self.mn_sp = 4;
        self.mn_sr = 1;
        self.mn_sy = 1;
        self.mn_er = 1;
        self.mn_ep = 1;
        self.mn_joint5 = 1;
        self.mn_joint6 = 1;

        self.global_counter = 0;
        self.mn_weight = 0.2 # the new value will be 0.3 
        # filter out very large torque 
        self.mn_fil_thr = 2
        
        self.torques = [0 for x in xrange(7)]
        self.velocities = [0 for x in xrange(7)]
        self.touch_sensor = []

        self.j = dict()
        self.f_tor = open('data/torque', 'w')
        self.f_vel = open('data/velocity', 'w')
        self.f_sensor = open('data/touch_sensor', 'w')

        # subscriber
        rospy.init_node('armTorqueListenAndTalk', anonymous=True)
        self.pub_torque = rospy.Publisher('torque_topic', Float64MultiArray)
        self.pub_velocity = rospy.Publisher('velocity_topic', Float64MultiArray)
        self.pub_dynamic_torque = rospy.Publisher('dynamic_torque_topic', Float64MultiArray)

        self.pub_torque_msg = std_msgs.msg.Float64MultiArray()
        self.pub_velocity_msg = std_msgs.msg.Float64MultiArray()
        self.pub_dynamic_torque_msg = std_msgs.msg.Float64MultiArray()
  

    def publishToFilteredTorque(self):
        ''' publish the filtered torque '''
        
        print "publisher"
        if time.time() - self.start > 2:
            
            for i in xrange(self.n):
                
                self.pub_msg.data = [float(self.sp_fil_[i]), float(self.sr_fil_[i]), float(self.sy_fil_[i]), 
                float(self.er_fil_[i]), float(self.ep_fil_[i]), float(self.joint5_fil_[i]), float(self.joint6_fil_[i])]
               
                self.pub.publish(self.msg)
               
                # print self.pub_msg 
            
                # self.writeToFile(self.msg)
            

    def callback(self, data):
        ''' callback for listening to the arm torques '''
        # get dynamic torque 
        self.dynamic_torque = data.dynamic_torque

        # print 'callback'
        self.end = time.time()
        self.global_counter += 1 

        t = np.subtract(np.array(data.total_torque),np.array(data.dynamic_torque))
        self.torques = np.subtract(t,np.array(data.trajectory_torque))

        self.sp.append(float(self.torques[0])) # push up - decrease  
        self.sr.append(float(self.torques[1])) # pushing inward - increase 
        self.sy.append(float(self.torques[2])) # twist cc - decrease 
        self.er.append(float(self.torques[3])) #s pushing outward - increase 
        self.joint5.append(float(self.torques[4]))
        self.joint6.append(float(self.torques[5]))
        self.ep.append(float(self.torques[6]))

        self.publishToTorque()
        # self.writeToFile(self.torques)
        # if int(self.end - self.start) % self.iterval == 6:
        #     self.visualize()

        #  start filtering after some time 
        if self.end - self.start > 1:
            self.counter += 1

            # filter torque and publish 
            # if self.counter == self.n:
            #     self.counter = 0
            #     (self.sp_fil_, self.mn_sp) = self.filter(self.sp[-self.n: ], self.factors[0], self.mn_sp)
            #     (self.sr_fil_, self.mn_sr) = self.filter(self.sr[-self.n: ],  self.factors[1], self.mn_sr)
            #     (self.sy_fil_, self.mn_sy) = self.filter(self.sy[-self.n: ],  self.factors[2], self.mn_sy)
            #     (self.er_fil_, self.mn_er) = self.filter(self.er[-self.n: ],  self.factors[3], self.mn_er)
            #     (self.ep_fil_, self.mn_ep) = self.filter(self.ep[-self.n: ],  self.factors[6], self.mn_ep)
            #     (self.joint5_fil_, self.mn_joint5) = self.filter(self.joint5[-self.n: ],  self.factors[4], self.mn_joint5)
            #     (self.joint6_fil_, self.mn_joint6) = self.filter(self.joint6[-self.n: ],  self.factors[5], self.mn_joint6)

            #     self.sp_fil += self.sp_fil_
            #     self.sr_fil += self.sr_fil_      
            #     self.sy_fil += self.sy_fil_ 
            #     self.er_fil += self.er_fil_
            #     self.ep_fil += self.ep_fil_ 
            #     self.joint5_fil += self.joint5_fil_ 
            #     self.joint6_fil += self.joint6_fil_ 
                
                # self.publishToFilteredTorque()

        # if int(self.end - self.start)%self.iterval == 5:
        #     self.visualize1()


            # r.sleep()
    def callback2(self, data):
        self.velocities = data.velocities

    def callback3(self, data):
        self.touch_sensor = data.data

    def listener(self):
        ''' listen to HERB's arm torques '''

        print "listener"
        topic = rospy.get_param('~topic', '/left/owd/waminternals')
        rospy.Subscriber(topic, owd_msgs.msg.WAMInternals, self.callback)

        topic2 = rospy.get_param('~topic2', '/left/owd/wamstate')
        rospy.Subscriber(topic2, owd_msgs.msg.WAMState, self.callback2)

        rospy.spin()


    def publishToTorque(self):
        ''' publish the raw torque '''

        # print "publisher raw "
        tor = []
        vel = []
        dyn = []

        r = rospy.Rate(10) # 10hz 

        if time.time() - self.start > 2:
            
            for i in xrange(7):
                tor.append(float(self.torques[i]) )
                vel.append(float(self.velocities[i]))
                dyn.append(float(self.dynamic_torque[i]))

            self.pub_torque_msg.data = tor 
            self.pub_torque.publish(self.pub_torque_msg)
            self.writeToFile(self.pub_torque_msg, self.f_tor)


            self.pub_velocity_msg.data = vel
            self.pub_velocity.publish(self.pub_velocity_msg)
            self.writeToFile(self.pub_velocity_msg, self.f_vel)

            self.pub_dynamic_torque_msg.data = self.dynamic_torque
            self.pub_dynamic_torque.publish(self.pub_dynamic_torque_msg)
            
            print self.pub_torque_msg
            print self.pub_velocity_msg
            print self.pub_dynamic_torque_msg

    def visualize(self):

        plt.figure(1)
        plt.subplot(711)
        plt.plot(self.sp, 'bo')
        
        plt.subplot(712)
        plt.plot(self.sr, 'bo')
        
        plt.subplot(713)
        plt.plot(self.sy, 'bo')
        
        plt.subplot(714)
        plt.plot(self.er, 'bo')
        
        plt.subplot(715)
        plt.plot(self.joint5, 'bo')
        
        plt.subplot(716)
        plt.plot(self.joint6, 'bo')
        
        plt.subplot(717)
        plt.plot(self.ep, 'bo')
        plt.show()
        time.sleep(1)
        plt.close()

    def visualize1(self):

        plt.figure(2)
        plt.subplot(711)
        plt.plot(self.sp_fil, 'bo')
        
        plt.subplot(712)
        plt.plot(self.sr_fil, 'bo')
        
        plt.subplot(713)
        plt.plot(self.sy_fil, 'bo')
        
        plt.subplot(714)
        plt.plot(self.er_fil, 'bo')
        
        plt.subplot(715)
        plt.plot(self.joint5_fil, 'bo')
        
        plt.subplot(716)
        plt.plot(self.joint6_fil, 'bo')
        
        plt.subplot(717)
        plt.plot(self.ep_fil, 'bo')
        plt.show()
        time.sleep(1)
        plt.close()

    def filter(self, points, factor, mn_glb):

        filteredPoints = []
        max_diff = 0.1 

        # reset global counter so it doesn't overflow 
        if self.global_counter >= 50:
            self.global_counter = 50

        mn_glb = (self.mn_weight * self.global_counter * mn_glb + (1 - self.mn_weight) * sum(points))/(self.global_counter + len(points)) 

        for p in points:
            
            if abs(p - mn_glb) > factor * mn_glb:
                filteredPoints.append(p)
            
            else:
                filteredPoints.append(mn_glb) 

        # store the global mean 
        return (filteredPoints, filteredPoints[-1])


    def writeToFile(self, sth, f):

        f.write(str(sth))
        f.write('\n')
        
# if __name__ == '__main__':
#     listener = ListenToArmTorque()
#     listener.listener()

