#!/usr/bin/env python
#Autores: Leonardo Gracida Munoz A0137, Nancy L.Garcia Jimenez A01378043
#Importamos las librerias de rospy y nimpy
import rospy
import numpy as np
#Importamos los mensajes de ROS necesarios
from std_msgs.msg import Float32,Bool
from geometry_msgs.msg import Pose2D,Twist

class Odom_mov():
    def __init__(self):
        #Iniciamos el nodo
        rospy.init_node("odom_mov")
        self.posicion = 0
        self.ed = 0
        self.eth = 0
        self.estado_sem = 0
        #Creamos los subscribers
        rospy.Subscriber("/pos",Pose2D,self.pos_callback)
        rospy.Subscriber("/ed",Float32,self.ed_callback)
        rospy.Subscriber("/eth",Float32,self.eth_callback)
        rospy.Subscriber("/estado_sem",Float32,self.sem_callback)
        #Creamos el publisher para poder mover el puzzlebot
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        self.pub_flag = rospy.Publisher("/cur_succ",Bool, queue_size=1)
        #Declaramos los mensajes por segundo
        self.rate = rospy.Rate(100)
        #Iniciamos un mensaje tipo Twist
        self.robot_cmd = Twist()
        #Cuando se acabe el codigo o lo tiremos llamemos esa funcion
        rospy.on_shutdown(self.end_callback)
    #funciones callback para extraer los datos de los suscriptores
    def pos_callback(self,data):
        self.posicion = [data.theta,data.x,data.y]
    def ed_callback(self,data):
        self.ed = data.data
    def eth_callback(self,data):
        self.eth = data.data
    def sem_callback(self,data):
        self.estado_sem = data.data
    #Funcion que va a para el robot cuando sea llamada
    def end_callback(self):
    	self.robot_cmd.linear.x = 0.0
    	self.robot_cmd.angular.z = 0.0
    	self.pub.publish(self.robot_cmd)

    def main(self):
        """Funcion que checa el error en angulo y distancia, usa un sistema
        de control proporcional y un filtro de saturacion para
        parar el carro al llegar a la posicion deseada."""
        while not rospy.is_shutdown():
            #establecimiento de la velocidad a partir del mensaje Twist que contiene la lineal y la angular
            velocidad = Twist()
            #-------
            #Error de distancia
            e_r_d = self.ed
            #Declaramos la ganancia proporcional
            kp = 1
            #aplicacion de control proporcional a la distancia
            control_d = e_r_d * kp
            lim_vel = 0.05
            lim_vel_ang = 0.1
            #establecimiento de filtro que limita la velocidad
            if control_d > lim_vel:
                control_d = lim_vel
            elif control_d < -1*lim_vel:
                control_d = -1*lin_vel
            #-------
            #Error de angulo
            e_r_a = self.eth
            #aplicacion de control proporcional al angulo
            control_a = e_r_a * kp
            if control_a > lim_vel_ang:
                control_a = lim_vel_ang
            elif control_a < -1*lim_vel_ang:
                control_a = -1*lim_vel_ang
            #-------
            """Declaramos la distancia minima entre el punto real y el del puzzlebot para declarar que llegamos al punto deseado,
            entre mas vuelta haga el puzzlebot el error entre estos puntos sera mayor."""
            if (e_r_d) < 0.155:
                #detener el robot en caso de que el error sea mayor a ese valor
                velocidad.linear.x = 0.0
                velocidad.angular.z = 0.0
                #Si llegamos cerca de la meta madamos un True
                flag = Bool()
                flag.data = True
                self.pub_flag.publish(flag)
            else:
                #aplicar el movimiento con control en caso de que el error no pase el limite
                #Multiplicamos el estado del semaforo por la velocidad del sistema de control
                velocidad.linear.x = control_d*self.estado_sem
                velocidad.angular.z = control_a*self.estado_sem
                #Si todavia no estamos cerca de la meta madanmos un False
                flag = Bool()
                flag.data = False
                self.pub_flag.publish(flag)
            #Publicamos la velocidad
            self.pub.publish(velocidad)
            #Declaramos el sleep para asegurar los mensaje por segundo.
            self.rate.sleep()
#Si el programa es corrido directamente
if __name__ == "__main__":
    #Creamos la clase
    odo = Odom_mov()
    #Iniciamos el programa
    odo.main()
