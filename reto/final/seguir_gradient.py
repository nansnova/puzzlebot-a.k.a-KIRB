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
        senal_anterior = 0
        while not rospy.is_shutdown():
            try:
                vals_centroide = []
                #Pasamos el frame a escalas de grises
                gray = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY)
                gray = gray[220:,:]
                gray_blur = cv.GaussianBlur(gray,(5,5),10)
                #Kernel para hacer operaciones morfologicas
                kernel = np.ones((3,3),np.uint8)
                erosion = cv.erode(gray_blur,kernel,iterations = 3)
                dilation = cv.dilate(erosion,kernel,iterations = 3)
                vert_sum = dilation.sum(axis=0)
                min_number = np.min(vert_sum)
                gradiante = np.gradient(vert_sum.astype('float32'))
                gradiante2 = np.gradient(vert_sum.astype('float32'),edge_order=2)
                gradiante_pos = (gradiante > 300) * gradiante
                gradiante_neg = (gradiante < -200) * gradiante
                multi_pos = gradiante_pos * gradiante2
                multi_neg = gradiante_neg * gradiante2
                multi_pos_left = np.roll(multi_pos,-1)
                multi_neg_left = np.roll(multi_neg,-1)
                resultante_der = []
                for i,j in zip(multi_pos,multi_pos_left):
                    if j > i:
                        resultante_der.append(j)
                resultante_izq = []
                for i,j in zip(multi_neg,multi_neg_left):
                    if j > i:
                        resultante_izq.append(j)
                    else:
                        resultante_izq.append(0)
                der = np.where(multi_pos_left > multi_pos)
                izq = np.where(multi_neg_left > multi_neg)
                if (len(izq[0]) != 0)and(len(der[0]) != 0):
                    izq_res = np.absolute(izq[0]-160)
                    index = np.where(izq_res == np.min(izq_res))[0][0]
                    izq = izq[0][index-1]
                    der_res = np.absolute(der[0]-160)
                    index = np.where(der_res == np.min(der_res))[0][0]
                    der = der[0][index-1]
                else:
                    izq = 0
                    der = 0
                #print("izquierda: ",izq)
                #print("derecha: ",der)
                ancho = abs(der-izq)
                print(ancho)
                #print("senal",self.class_det)
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
                #print(abs(error),err_x)
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
                    if (err_x>120)and(err_x<200)and(senal_guardada == 4):
                        estado_giro = False
                        estado_switch = False
                    if (ancho > 30)and(senal_guardada == 2):
                        estado_giro = False
                        estado_switch = False

                    if senal_guardada == 6:
                        pub_giro.data = 3
                        self.pub_giro.publish(pub_giro)

                #publicarlo
                self.pub_error.publish(error)
                senal_anterior = senal_guardada
            except:
                #En caso de no recibir imagen imprimimos vacio
                print("vacio")
            #Aseguramos los Mensajes por segundo deseados
            self.rate.sleep()
if __name__ == "__main__":
    img = getBlackLine()
    img.main()