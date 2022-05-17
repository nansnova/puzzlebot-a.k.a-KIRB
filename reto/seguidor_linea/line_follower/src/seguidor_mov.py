#!/usr/bin/env python
#Autores: Leonardo Gracida Munoz A0137, Nancy L.Garcia Jimenez A01378043
#Importamos las librerias de rospy y numpy
import rospy
import numpy as np
#Importamos los mensajes de ROS necesarios
from std_msgs.msg import Float32,Bool
from geometry_msgs.msg import Pose2D,Twist

class SeguidorMov():
    def __init__(self):
        #Iniciamos el nodo
        rospy.init_node("seguidor_mov")
        self.err_line = 0
        self.con_giro = 0
        #Creamos los subscribers
        rospy.Subscriber("/err_line",Float32,self.err_line_callback)
        rospy.Subscriber("/con_giro",Float32,self.giro_callback)
        #Creamos el publisher para poder mover el puzzlebot
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        #Declaramos los mensajes por segundo
        self.rate = rospy.Rate(100)
        #Iniciamos un mensaje tipo Twist
        self.robot_cmd = Twist()
        #Cuando se acabe el codigo o lo tiremos llamemos esa funcion
        rospy.on_shutdown(self.end_callback)
    #funciones callback para extraer los datos de los suscriptores
    def err_line_callback(self,data):
        self.err_line = data.data
    #Numero que controla la velocidad lineal del robot
    def giro_callback(self,data):
        self.con_giro = data.data
    #Funcion que va a para el robot cuando sea llamada
    def end_callback(self):
    	self.robot_cmd.linear.x = 0.0
    	self.robot_cmd.angular.z = 0.0
    	self.pub.publish(self.robot_cmd)

    def main(self):
        while not rospy.is_shutdown():
            #Declaramos la ganancia proporcional
            kp = 0.003
            lim_vel_ang = 0.3
            #Error de angulo
            err_line = self.err_line
            #aplicacion de control proporcional al angulo
            proporcional = err_line * kp
            #Filtro de saturacion
            if proporcional > lim_vel_ang:
                proporcional = lim_vel_ang
            elif proporcional < -1*lim_vel_ang:
                proporcional = -1*lim_vel_ang
            #En caso de ir recto aumentar la velocidad
            if (proporcional < 0.05) and (proporcional > -0.05):
                boost = 1.2
            else:
                boost = 1
            self.robot_cmd.angular.z = proporcional
            #Aplicamos la variables de control de velocidad lineal y el aumento de velocidad
            self.robot_cmd.linear.x = 0.2 * self.con_giro * boost
            #Publicamos la velocidad
            self.pub.publish(self.robot_cmd)
            #Declaramos el sleep para asegurar los mensaje por segundo.
            self.rate.sleep()
#Si el programa es corrido directamente
if __name__ == "__main__":
    #Creamos la clase
    seguidor_con = SeguidorMov()
    #Iniciamos el programa
    seguidor_con.main()
