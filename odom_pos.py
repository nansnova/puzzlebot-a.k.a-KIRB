#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D

class Odom():
    def __init__(self):
        rospy.init_node("odom_pos")
        self.wr = 0
        self.wl = 0
        self.th = 0.0
        rospy.Subscriber("/wr",Float32,self.wr_callback)
        rospy.Subscriber("/wl",Float32,self.wl_callback)
        self.posicion= rospy.Publisher("/pos", Pose2D, queue_size=1)
        self.ed = rospy.Publisher("/ed", Float32, queue_size = 1)
        self.eth = rospy.Publisher("/eth", Float32, queue_size = 1)
        self.pos = np.array([[0.0],[0.0],[0.0]])
        self.rate = rospy.Rate(2000)
    def wr_callback(self,w):
        self.wr = w.data
    def wl_callback(self,w):
        self.wl = w.data
    def cal_odom(self,dt):
        w = 0.05*((self.wr-self.wl)/0.19)
        v = 0.05*((self.wr+self.wl)/2)
        #print(w)
        #print(v)
        y_k = self.pos
        if dt < 1:
            y_k1 = y_k + (np.array([[w],[v*np.cos(y_k[0,0])],[v*np.sin(y_k[0,0])]]))*dt
        else:
            y_k1 = y_k
        if (y_k1[0,0] < 0.0):
            y_k1[0,0] = y_k1[0,0] + 2*np.pi
        if (y_k1[0,0] > 2*np.pi):
            y_k1[0,0] = y_k1[0,0] - 2*np.pi
        self.pos = y_k1
        return y_k1
    def main(self,x,y):
        t0 = rospy.get_rostime().to_sec()
        while not rospy.is_shutdown():
            pos = Pose2D()
            eth = Float32()
            ed = Float32()
            tf = rospy.get_rostime().to_sec()
            y_k = self.cal_odom(tf-t0)
            #print(y_k)
            eth.data = np.arctan2(y,x)-y_k[0,0]
            ed.data = ((x-y_k[1,0])**2 + (y - y_k[2,0])**2)**0.5
            #print(np.sqrt(((x-y_k[1,0])**2 + (y - y_k[2,0]))))
            #print(eth.data)
            #print(ed.data)
            pos.x = y_k[1,0]
            pos.y = y_k[2,0]
            pos.theta = y_k[0,0]
            self.posicion.publish(pos)
            self.eth.publish(eth)
            self.ed.publish(ed)
            self.rate.sleep()
            t0 = tf
if __name__ == "__main__":
    mov = Odom()
    mov.main(1,1)
