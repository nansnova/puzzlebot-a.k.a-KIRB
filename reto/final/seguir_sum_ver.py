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
        rospy.Subscriber("/camera/image_raw",Image,self.img_callback)
        rospy.Subscriber("/wr",Float32,self.wr_callback)
        rospy.Subscriber("/wl",Float32,self.wl_callback)
        #Creamos los Publishers
        self.pub = rospy.Publisher("/img_line_res", Image, queue_size = 10)
        self.pub_con_giro = rospy.Publisher("/con_giro", Float32, queue_size = 1)
        self.pub_error = rospy.Publisher("/err_line", Float32, queue_size = 10)
        #Variables usadas para obtener los datos de los subscribers
        self.frame = np.array([[]],dtype = "uint8")
        self.wr = 0
        self.wl = 0
        #Creamos el bridge de cv2 a ROS smg y viceversa
        self.bridge = CvBridge()
        #Mensajes por segundo
        self.rate = rospy.Rate(60)

    #Callbacks para obetener la informacion de los topicos
    def img_callback(self,data):
        self.frame = self.bridge.imgmsg_to_cv2(data,desired_encoding = "passthrough")
    def wr_callback(self,data):
        self.wr = data.data
    def wl_callback(self,data):
        self.wl = data.data

    def main(self):
        while not rospy.is_shutdown():
            try:
                #Pasamos el frame a escalas de grises
                gray = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY)
                gray = cv.resize(gray,(320,280),interpolation = cv.INTER_NEAREST)
                gray = gray[160:,:]
                gray_blur = cv.GaussianBlur(gray,(5,5),0)
                #Kernel para hacer operaciones morfologicas
                kernel = np.ones((3,3),np.uint8)
                erosion = cv.erode(gray_blur,kernel,iterations = 3)
                dilation = cv.dilate(erosion,kernel,iterations = 3)
                #suma vertical (320 elementos)
                vert_sum = dilation.sum(axis=0) #0 black 255 white
                #obtener punto min (val min)
                min_number = min(vert_sum)
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
                print(error)
                #publicarlo
                self.pub_error.publish(error)
                dilation = cv.cvtColor(dilation,cv.COLOR_GRAY2BGR)
                #pintar el centroide
                cv.circle(dilation,(err_x,80),3,(0,0,255),1)
                img = self.bridge.cv2_to_imgmsg(dilation)
                img.encoding = "bgr8"
                self.pub.publish(img)
                #suscribir topicos de movimiento


            except:
                #En caso de no recibir imagen imprimimos vacio
                print("vacio")
            #Aseguramos los Mensajes por segundo deseados
            self.rate.sleep()
if __name__ == "__main__":
    img = getBlackLine()
    img.main()
