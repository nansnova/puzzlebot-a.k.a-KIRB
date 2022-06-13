#!/usr/bin/env python
#Autores: Leonardo Gracida Munoz A0137, Nancy L.Garcia Jimenez A01378043
#Importamos las librerias de rospy, numpy y cv2
import rospy
import cv2 as cv
import numpy as np
#Importamos los mensjes necesarios
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
#Importamos el Bridge de cv2
from cv_bridge import CvBridge

class getBlackLine():
    def __init__(self):
        #Iniciamos el nodo
        rospy.init_node("get_black_line")
        #Creamos los subscribers necesarios
        rospy.Subscriber("/video_source/raw",Image,self.img_callback)
        rospy.Subscriber("/class_det",Float32,self.class_det_callback)
        #Creamos los Publishers
        self.pub_error = rospy.Publisher("/err_line", Float32, queue_size = 10)
        self.pub_giro = rospy.Publisher("/edo_giro", Float32, queue_size = 1)
        self.classDictionary = {6: "stop", 1: "fin_prob", 2: "derecha", 3: "izquierda", 4: "siga", 5: "rotonda"}
        #Variables usadas para obtener los datos de los subscribers
        self.frame = np.array([[]],dtype = "uint8")
        self.class_det = 0
        #Creamos el bridge de cv2 a ROS smg y viceversa
        self.bridge = CvBridge()
        #Mensajes por segundo
        self.rate = rospy.Rate(60)

    #Callbacks para obetener la informacion de los topicos
    def img_callback(self,data):
        self.frame = self.bridge.imgmsg_to_cv2(data,desired_encoding = "passthrough")
    def class_det_callback(self,data):
        self.class_det = data.data

    def main(self):
        estado_switch = False
        senal_guardada = 0
        estado_giro = False
        pub_giro = Float32()
        while not rospy.is_shutdown():
            try:
                vals_centroide = []
                #Pasamos el frame a escalas de grises
                gray = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY)
                gray = gray[210:,:]
                gray_blur = cv.GaussianBlur(gray,(5,5),0)
                #Kernel para hacer operaciones morfologicas
                kernel = np.ones((3,3),np.uint8)
                erosion = cv.erode(gray_blur,kernel,iterations = 3)
                dilation = cv.dilate(erosion,kernel,iterations = 3)
                #suma vertical (320 elementos)
                vert_sum = dilation.sum(axis=0) #0 black 255 white
                #obtener punto min (val min)
                min_number = min(vert_sum)
                vals_centroide.append(min_number)

                rep = {}
                for r in vals_centroide:
                    if vals_centroide.count(r) !=1:
                        if min_number in rep:
                            rep[min_number] += 1
                        else:
                            rep[min_number] = 0
                if (self.class_det == 0):
                    pub_giro.data = 0
                    self.pub_giro.publish(pub_giro)
                if (self.class_det != 0)and(estado_switch == False)and(self.class_det != 5)and(self.class_det != 1):
                    #print(self.classDictionary[self.class_det])
                    estado_switch = True
                    senal_guardada = self.class_det
                if estado_switch == True:
                    print(self.classDictionary[senal_guardada])

                #obtencion de la posicion
                err_x = 0
                pos = 0
                for num in vert_sum:
                    if num == min_number:
                        err_x = pos
                    else:
                        pos+=1
                #obtencion del error
                pos_x = int(gray.shape[1]/2)
                error = pos_x-err_x
                print(abs(error),err_x)
                if (abs(error)>80)and(estado_switch==True):
                    estado_giro = True
                if estado_giro == True:
                    print(abs(error),err_x)
                    if senal_guardada == 4:
                        error=0
                    if senal_guardada == 2:
                        pub_giro.data = 1
                        self.pub_giro.publish(pub_giro)
                    else:
                        pub_giro.data = 0
                        self.pub_giro.publish(pub_giro)
                    if (err_x>120)and(err_x<200):
                        estado_giro = False
                        estado_switch = False
                    if senal_guardada == 6:
                        pub_giro.data = 3
                        self.pub_giro.publish(pub_giro)

                #publicarlo
                self.pub_error.publish(error)

            except:
                #En caso de no recibir imagen imprimimos vacio
                print("vacio")
            #Aseguramos los Mensajes por segundo deseados
            self.rate.sleep()
if __name__ == "__main__":
    img = getBlackLine()
    img.main()
