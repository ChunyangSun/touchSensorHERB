#!/usr/bin/env python

# listener and subscriber  
import rospy, std_msgs.msg
from std_msgs.msg import Float64MultiArray

# herbpy
import roslib
roslib.load_manifest("touchSensor")
import owd_msgs.msg, owd_msgs.msg._WAMState

# visualization
import matplotlib.pyplot as plt

# others 
import time
from IPython import embed 
from numpy import * 
from numpy.linalg import inv 

class ListenToArmTorque():
    def __init__(self, planner= None):
        self.planner = planner 
        self.f = open('wamstate', 'r+')
        self.start = time.time()
        self.thr = 1
        self.iterval = 8
        self.n = 50
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
        self.c_sp = 0.3
        self.c_sr = 0.2
        self.c_sy = 0.1
        self.c_er = 0.2
        self.c_ep = 0.2
        self.c_joint5 = 0.1
        self.c_joint6 = 0.2
        self.mn_sp = 4;
        self.mn_sr = 1;
        self.mn_sy = 1;
        self.mn_er = 1;
        self.mn_ep = 1;
        self.mn_joint5 = 1;
        self.mn_joint6 = 1;

        # filter out very large torque 
        self.mn_fil_thr = 2

        self.j = dict()

        self.f = open('torque_log_listen', 'w')

        # subscriber
        rospy.init_node('armTorqueListenAndTalk', anonymous=True)
         
        # publisher
        self.pub = rospy.Publisher('fil_torque_topic', Float64MultiArray)
        self.r = rospy.Rate(10) # 10hz 
        self.msg = std_msgs.msg.Float64MultiArray()
 
    def publishToFilteredTorque(self):
        ''' publish the filtered torque '''
        
        print "publisher"
        if time.time() - self.start > 2:
            
            for i in xrange(self.n):
                
                self.msg.data = [float(self.sp_fil_[i]), float(self.sr_fil_[i]), float(self.sy_fil_[i]), 
                float(self.er_fil_[i]), float(self.ep_fil_[i]), float(self.joint5_fil_[i]), float(self.joint6_fil_[i])]
               
                self.pub.publish(self.msg)
               
                print self.msg 
            
                self.writeToFile()
            
            self.r.sleep()

    def callback(self, data):
        ''' callback for listening to the arm torques '''

        self.end = time.time()
        self.torques = data.torques

        self.sp.append(float(self.torques[0])) # push up - decrease  
        self.sr.append(float(self.torques[1])) # pushing inward - increase 
        self.sy.append(float(self.torques[2])) # twist cc - decrease 
        self.er.append(float(self.torques[3])) #s pushing outward - increase 
        self.joint5.append(float(self.torques[4]))
        self.joint6.append(float(self.torques[5]))
        self.ep.append(float(self.torques[6]))

        # if int(self.end - self.start) % self.iterval == 7:
        #     self.visualize()

        # only start filtering after some time 
        if self.end - self.start > 1:
            self.counter += 1
            if self.counter == self.n:
                (self.sp_fil_, self.mn_sp) = self.filter(self.sp[-self.n: ], self.c_sp, self.mn_sp)
                (self.sr_fil_, self.mn_sr) = self.filter(self.sr[-self.n: ],  self.c_sr, self.mn_sr)
                (self.sy_fil_, self.mn_sy) = self.filter(self.sy[-self.n: ],  self.c_sy, self.mn_sy)
                (self.er_fil_, self.mn_er) = self.filter(self.er[-self.n: ],  self.c_er, self.mn_er)
                (self.ep_fil_, self.mn_ep) = self.filter(self.ep[-self.n: ],  self.c_ep, self.mn_ep)
                (self.joint5_fil_, self.mn_joint5) = self.filter(self.joint5[-self.n: ],  self.c_joint5, self.mn_joint5)
                (self.joint6_fil_, self.mn_joint6) = self.filter(self.joint6[-self.n: ],  self.c_joint6, self.mn_joint6)

                self.sp_fil += self.sp_fil_
                self.sr_fil += self.sr_fil_      
                self.sy_fil += self.sy_fil_ 
                self.er_fil += self.er_fil_
                self.ep_fil += self.ep_fil_ 
                self.joint5_fil += self.joint5_fil_ 
                self.joint6_fil += self.joint6_fil_ 

                self.counter = 0
                self.publishToFilteredTorque()
        
        if int(self.end - self.start)%self.iterval == 10:
            self.visualize1()

    def listener(self):
        ''' listen to HERB's arm torques'''

        print "listener"
        topic = rospy.get_param('~topic', '/left/owd/wamstate')
        
        rospy.Subscriber(topic, owd_msgs.msg.WAMState, self.callback)
        rospy.spin()

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

        # print len(self.sp_fil)

        plt.figure(2)
        plt.subplot(711)
        plt.plot(self.sp_fil_, 'bo')
        
        plt.subplot(712)
        plt.plot(self.sr_fil_, 'bo')
        
        plt.subplot(713)
        plt.plot(self.sy_fil_, 'bo')
        
        plt.subplot(714)
        plt.plot(self.er_fil_, 'bo')
        
        plt.subplot(715)
        plt.plot(self.joint5_fil_, 'bo')
        
        plt.subplot(716)
        plt.plot(self.joint6_fil_, 'bo')
        
        plt.subplot(717)
        plt.plot(self.ep_fil_, 'bo')
        plt.show()
        time.sleep(1)
        plt.close()

    def filter(self, points, factor, mn_glb):
        filteredPoints = []
        max_diff = 0 
        # take mean 
        mn = sum(points)*1.0/self.n
        # get the global value from last batch  
        mn_fil = mn_glb

        for p in points:
        
            # update the max difference 
            max_diff = max(max_diff, abs(p-mn))

            # print self.mn_fil_thr*mn_fil
            # low pass filter: the absolute value cannot be larger than mean 
            if abs(p - mn) < factor * abs(max_diff):
                filteredPoints.append(p)
            else:
                filteredPoints.append(mn_fil)
            

            # update the mean after adding each point 
            mn_fil = sum(filteredPoints)*1.0/len(filteredPoints)
        
        # store the global mean 
        return (filteredPoints, filteredPoints[-1])

    def writeToFile(self):

        self.f.write(str(self.msg))
        self.f.write('\n')
        
# if __name__ == '__main__':
#     listener = ListenToArmTorque()
#     listener.listener()

