#!/usr/bin/env python
import rospy
import std_msgs.msg
import roslib
roslib.load_manifest("touchSensor")

import owd_msgs.msg
import owd_msgs.msg._WAMState

#visualization
import matplotlib.pyplot as plt
import time
from IPython import embed 

from numpy import * 
from numpy.linalg import inv 

class listenToArmTorque():
    def __init__(self):
        self.f = open('wamstate', 'r+')
        self.torqueList = []
        self.start = time.time()
        self.thr = 1
        self.sp_fil = []
        self.iterval = 5


    def callback(self, data):
        self.end = time.time()
        # print self.end - self.start
        torques = data.torques
        
        self.torqueList.append(torques)
        torques = str(torques)
        self.f.write(torques)
        self.f.write("\n")

        self.sp = [float(tpl[0]) for tpl in self.torqueList] # push up - decrease  
        self.sr = [float(tpl[1]) for tpl in self.torqueList] # pushing inward - increase 
        self.sy = [float(tpl[2]) for tpl in self.torqueList] # twist cc - decrease 
        self.er = [float(tpl[3]) for tpl in self.torqueList] # pushing outward - increase 
        self.joint5 = [float(tpl[4]) for tpl in self.torqueList]
        self.joint6 = [float(tpl[5]) for tpl in self.torqueList]
        self.ep = [float(tpl[6]) for tpl in self.torqueList]
        
        if len(self.sp) > 5:
            for i in xrange(len(self.sp)- 2):
                self.sp_fil = self.sp_fil + self.filter(self.sp[i:i+3], self.thr)
        
        # if int(self.end - self.start)%self.iterval == 0:
        #     self.visualize()

    def listener(self):
        rospy.init_node('armTorqueListener', anonymous=True)
        topic = rospy.get_param('~topic', '/left/owd/wamstate')
        
        rospy.Subscriber(topic, owd_msgs.msg.WAMState, self.callback)
        rospy.spin()


    def visualize(self):
        print "hello visualize"
        print len(self.sp_fil)

        plt.figure(1)
        plt.subplot(711)
        plt.plot(self.sp_fil, 'bo')
        
        plt.subplot(712)
        plt.plot(self.sr, 'bo')
        
        plt.subplot(713)
        plt.plot(self.sy, 'bo')
        
        plt.subplot(714)
        plt.plot(self.ep, 'bo')
        
        plt.subplot(715)
        plt.plot(self.joint5, 'bo')
        
        plt.subplot(716)
        plt.plot(self.joint6, 'bo')
        
        plt.subplot(717)
        plt.plot(self.ep, 'bo')
        plt.show()

    # def filter(self, datapoints, thr):
    #     N_iter = 20
    #     for i in arange(0, N_iter): 
    #         (X, P) = kf_predict(X, P, A, Q, B, U) 
    #         (X, P, K, IM, IS, LH) = kf_update(X, P, Y, H, R) 
    #         Y = array([[X[0,0] + abs(0.1 * randn(1)[0])],[X[1, 0] + abs(0.1 * randn(1)[0])]]) 


    # def kf_update(X, P, Y, H, R): 
    #  IM = dot(H, X) 
    #  IS = R + dot(H, dot(P, H.T)) 
    #  K = dot(P, dot(H.T, inv(IS))) 
    #  X = X + dot(K, (Y-IM)) 
    #  P = P - dot(K, dot(IS, K.T)) 
    #  LH = gauss_pdf(Y, IM, IS) 
    #  return (X,P,K,IM,IS,LH) 

    # def gauss_pdf(X, M, S): 
    #  if M.shape()[1] == 1: 
    #      DX = X - tile(M, X.shape()[1]) 
    #      E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0) 
    #      E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S)) 
    #      P = exp(-E) 
    #  elif X.shape()[1] == 1: 
    #      DX = tile(X, M.shape()[1])- M 
    #      E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0) 
    #      E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S)) 
    #      P = exp(-E) 
    #  else: 
    #      DX = X-M 
    #      E = 0.5 * dot(DX.T, dot(inv(S), DX)) 
    #      E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S)) 
    #      P = exp(-E) 
    #  return (P[0],E[0]) 


if __name__ == '__main__':
    listener = listenToArmTorque()
    listener.listener()

