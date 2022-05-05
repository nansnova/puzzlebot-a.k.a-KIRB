#!/usr/bin/env python
import rospy
import numpy as np
#Importamos los mensajes de ROS necesarios
from std_msgs.msg import Float32,Bool
from geometry_msgs.msg import Pose2D,Twist

class ChangePos():
    def __init__(self):
        rospy.init_node("change_pos")
        rospy.Subscriber("/cur_succ",Bool,self.cur_succ_callback) #current pose
        self.posicion_pub = rospy.Publisher("/curPos", Pose2D, queue_size=1)
        self.posiciones = [(1,1),(1,0.5),(1.5,0)]
        self.curr_succ = False
        self.rate = rospy.Rate(100)

        #rospy.on_shutdown(self,end_callback)

    def cur_succ_callback(self,data):
        self.curr_succ = data.data

    def main(self):
        """
        Funcion
        """
        while not rospy.is_shutdown():
            pos_index = 0
            posicion = Pose2D()
            limite = len(self.posiciones)-1
            if pos_index == limite:
                continue
            if self.curr_succ == True:
                pos_index += 1


            x,y = self.posiciones[pos_index]
            posicion.x = x
            posicion.y = y
            self.posicion_pub.publish(posicion)
            self.rate.sleep()

#Si el programa es corrido directamente
if __name__ == "__main__":
    #Creamos la clase
    pos = ChangePos()
    #Iniciamos el programa
    pos.main()
