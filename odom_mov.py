#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D,Twist

class Odom_mov():
    def __init__(self):
        rospy.init_node("odom_mov")
        self.posicion = 0
        self.ed = 0
        self.eth = 0
        rospy.Subscriber("/pos",Pose2D,self.pos_callback)
        rospy.Subscriber("/ed",Float32,self.ed_callback)
        rospy.Subscriber("/eth",Float32,self.eth_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        self.rate = rospy.Rate(100)
        self.robot_cmd = Twist()
        rospy.on_shutdown(self.end_callback)
    def pos_callback(self,data):
        self.posicion = [data.theta,data.x,data.y]
    def ed_callback(self,data):
        self.ed = data.data
    def eth_callback(self,data):
        self.eth = data.data
    def end_callback(self):
    	self.robot_cmd.linear.x = 0.0
    	self.robot_cmd.angular.z = 0.0
    	self.pub.publish(self.robot_cmd)
    def main(self):
        while not rospy.is_shutdown():
            velocidad = Twist()
            #-------
            R_d = 0
            e_r_d = self.ed
            kp = 1
            control_d = e_r_d * kp
            if control_d > 0.1:
                control_d = 0.1
            elif control_d < -0.1:
                control_d = -0.1
            #-------
            R_a = 0
            e_r_a = self.eth
            control_a = e_r_a * kp
            if control_a > 0.1:
                control_a = 0.1
            elif control_a < -0.1:
                control_a = -0.1
            #-------
            if (e_r_d) < 0.1:
                velocidad.linear.x = 0.0
                velocidad.angular.z = 0.0
            else:
                velocidad.linear.x = control_d
                velocidad.angular.z = control_a
            print(e_r_d,self.eth,control_d)
            self.pub.publish(velocidad)
            #print("ed",self.ed)
            #print("eth",self.eth)
            self.rate.sleep()
if __name__ == "__main__":
    odo = Odom_mov()
    odo.main()
