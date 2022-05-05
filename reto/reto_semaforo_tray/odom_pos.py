#!/usr/bin/env python
#Autores: Leonardo Gracida Munoz A01379812, Nancy Lesly Garcia Jimenez A01378043
#Importamos las librerias de rospy numpy y los tipos de mensajes necesarios
import rospy
import numpy as np
from std_msgs.msg import Float32,Bool
from geometry_msgs.msg import Pose2D

class Odom():
    def __init__(self):
        #Inicializar el nodo
        rospy.init_node("odom_pos")
        #crear las variables que conectan los valores de los suscribers
        self.wr = 0
        self.wl = 0
        self.th = 0.0
        #Creamos los Subscribers y Publisher necesarios
        rospy.Subscriber("/wr",Float32,self.wr_callback)
        rospy.Subscriber("/wl",Float32,self.wl_callback)
        rospy.Subscriber("/cur_succ",Bool,self.cur_succ_callback)
        #Publicamos la posicion en el espacio
        self.posicion = rospy.Publisher("/pos", Pose2D, queue_size=1)
        #Publicamos el error en angulo y distancia
        self.ed = rospy.Publisher("/ed", Float32, queue_size = 1)
        self.eth = rospy.Publisher("/eth", Float32, queue_size = 1)
        #Iniciamos el vector de posicion
        self.pos = np.array([[0.0],[0.0],[0.0]])
        #Ciclo de posiciones en el espacio a ejecutar
        self.posiciones = [(0,0),(2,0),(2,2),(0,2)]
        self.curr_succ = False
        #Declaramos los mensajes por segundo
        self.rate = rospy.Rate(200)

    #funciones callback para extraer los datos de los subscriber
    def wr_callback(self,w):
        self.wr = w.data
    def wl_callback(self,w):
        self.wl = w.data
    def cur_succ_callback(self,data):
        self.curr_succ = data.data

    #calculo de la odometria estimada
    def cal_odom(self,dt,x,y):
        """Funcion para hacer el calculo del vector de odometria a lo largo del tiempo"""
        #calculo de velocidades linear y angular
        w = 0.05*((self.wr-self.wl)/0.19)
        v = 0.05*((self.wr+self.wl)/2)
        y_k = self.pos
        #Solo hacemos el calculo y dt es menor a uno, evitando bugs
        if dt < 1:
            #En este calculo agregamos una suma de segundo orden al angulo usado en el diferencial de posicion en x y
            y_k1 = y_k + (np.array([[w],[v*np.cos(y_k[0,0])+(w*dt)/2],[v*np.sin(y_k[0,0])+(w*dt)/2]]))*dt
        else:
            y_k1 = y_k
        #En caso de querer tener un giro de 180 grados conforme al eje original
        if (x < 0.0) and (y == 0.0):
            #Pasamos los angulos calculados de positivos de 0 a 360
            if (y_k1[0,0] < 0.0):
                y_k1[0,0] = y_k1[0,0] + 2*np.pi
            if (y_k1[0,0] >= 2*np.pi):
                y_k1[0,0] = 0.0
        else:
            #En caso de estar muy ceca de 180 grados hacer el angulo directo a -180
            if y_k1[0,0] > np.radians(179.5):
                y_k1[0,0] = -1*np.pi
            #reiniciar el angulo si pasa el rango de -pi a pi
            if (y_k1[0,0] < -1*np.pi):
                y_k1[0,0] = y_k1[0,0] + 2*np.pi
            if (y_k1[0,0] > np.pi):
                y_k1[0,0] = y_k1[0,0] - 2*np.pi
        #Actualizamos el vector de posicion
        self.pos = y_k1

    def main(self):
        """Funcion que calcula el error de angulo y distancia y publica en los topicos"""
        #encontrar el tiempo inicial
        t0 = rospy.get_rostime().to_sec()
        #index de la posicion deseada
        pos_index = 0
        #Index limite de posiciones
        limite = len(self.posiciones)-1
        estado_call = False
        while not rospy.is_shutdown():
            #Si recibimos true cambiamos el index de la posicion deseada
            if self.curr_succ == True:
                #print(True)
                #Hasta que no recivamos un False vamos a actualizar el index
                if (pos_index <= limite) and (estado_call == False):
                    pos_index += 1
                    estado_call = True
                #Si pasamos el limite reiniciamos todo
                if (pos_index > limite):
                    pos_index = 0
            else:
                #Si recibimos un False podemos cambiar de posicion deseada.
                estado_call = False
            pos_index_sig = pos_index+1
            #Si la index actual es la ultima la index siguiente es la primera
            if (pos_index+1) == limite + 1:
                pos_index_sig = 0
            #Obtenemos posicion sigueinte y actual
            x_1,y_1 = self.posiciones[pos_index_sig]
            x,y = self.posiciones[pos_index]
            #Pose2D es un mensaje que toma x,y,z
            #Iniciamos los mensajes a usar
            pos = Pose2D()
            eth = Float32()
            #Calculamos el error de angulo
            ed = Float32()
            eth.data = np.arctan2(y_1 - y,x_1 - x)-self.pos[0,0]
            #Calculamos el error de distancia
            ed.data = np.sqrt((x_1-self.pos[1,0])**2 + (y_1 - self.pos[2,0])**2)

            #Ingresamoa la pisicion en x y theta
            pos.x = self.pos[1,0]
            pos.y = self.pos[2,0]
            pos.theta = self.pos[0,0]
            #se publican las posiciones y los errores
            self.posicion.publish(pos)
            self.eth.publish(eth)
            self.ed.publish(ed)
            #obtenemos el timepo final
            tf = rospy.get_rostime().to_sec()
            #Calculamos la odometria y actualizamos el vector de posicion
            self.cal_odom(tf-t0,x_1 - x,y_1 - y)
            self.rate.sleep()
            #actualizacion del tiempo
            t0 = tf
#Si el programa es corrido directamente
if __name__ == "__main__":
    #Iniciamos la clase
    mov = Odom()
    #Ingresamos el punto al que queremos ir en el espacio
    # X Y
    #mov.main(1,1)
    mov.main()
