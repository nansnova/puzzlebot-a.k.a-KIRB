#!/usr/bin/env python
#Autores: Leonardo Gracida Munoz A0137, Nancy L.Garcia Jimenez A01378043
#Importamos las librerias de rospy y nimpy
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
        self.tiempo = 0
        self.con_giro = 0
        #Creamos los subscribers
        rospy.Subscriber("/err_line",Float32,self.err_line_callback)
        rospy.Subscriber("/tiempo",Float32,self.tiempo_callback)
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
    def tiempo_callback(self,data):
        self.tiempo = data.data
    def giro_callback(self,data):
        self.con_giro = data.data
    #Funcion que va a para el robot cuando sea llamada
    def end_callback(self):
    	self.robot_cmd.linear.x = 0.0
    	self.robot_cmd.angular.z = 0.0
    	self.pub.publish(self.robot_cmd)

    def main(self):
        t0 = 0
        integral = 0
        derivado = 0
        err_ant = 0
        while not rospy.is_shutdown():
            #Declaramos la ganancia proporcional
            kp = 0.003
            lim_vel_ang = 0.35
            #Error de angulo
            err_line = self.err_line
            #aplicacion de control proporcional al angulo
            proporcional = err_line* kp
            tf = self.tiempo
            ki = 0.0004
            integral = integral + ki*err_line*(tf-t0)
            kd = 0.004
            if (tf-t0) != 0:
                derivado = kd * ((err_line - err_ant)/(tf-t0))
            PID = proporcional# + integral + derivado
            if PID > lim_vel_ang:
                PID = lim_vel_ang
            elif PID < -1*lim_vel_ang:
                PID = -1*lim_vel_ang
	    if (PID < 0.05) and (PID > -0.05):
		boost = 1.2
	    else:
		boost = 1
            #print(PID)
            self.robot_cmd.angular.z = PID
            self.robot_cmd.linear.x = 0.15 * self.con_giro * boost
            #Publicamos la velocidad
            self.pub.publish(self.robot_cmd)
            #Declaramos el sleep para asegurar los mensaje por segundo.
            t0 = tf
            err_ant = err_line
            self.rate.sleep()
#Si el programa es corrido directamente
if __name__ == "__main__":
    #Creamos la clase
    seguidor_con = SeguidorMov()
    #Iniciamos el programa
    seguidor_con.main()
