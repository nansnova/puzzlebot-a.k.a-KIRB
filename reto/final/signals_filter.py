#!/usr/bin/env python
#Autores:
#Leonardo Gracida Munoz A01379812
#Nancy L. García Jiménez A01378043

#Importamos las librerias deseadas
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

import os, rospkg
#from tensorflow.keras.models import load_model
import os
class Imagen():
    def __init__(self):
        rospy.init_node("signal_detector")
        rospy.Subscriber("/video_source/raw",Image,self.img_callback)
        self.pub_class = rospy.Publisher("/class_det", Float32, queue_size = 1)
        self.pub_img = rospy.Publisher("/signal_img", Image, queue_size = 1)
        self.bridge = CvBridge()
        self.rp = rospkg.RosPack()
        self.frame = np.array([[]],dtype = "uint8")
        self.script_path = os.path.join(self.rp.get_path("pista_manchester"), "src", "modelos","signals_5_ligero_red")
        self.classDictionary = {6: "stop", 1: "fin_prob", 2: "derecha", 3: "izquierda", 4: "siga", 5: "rotonda"}
        self.imageSize = (32, 32)
        self.rate = rospy.Rate(60)
    def img_callback(self,msg):
        #callback del punte entre ros y cv2
        self.frame = self.bridge.imgmsg_to_cv2(msg,desired_encoding = "passthrough")
    def preprocess(self,img):
        img = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
        img = cv.resize(img,(32,32))
        img = cv.equalizeHist(img)
        img = img/255
        img = np.expand_dims(img, axis=0)
        img = np.transpose(img, (1, 2, 0))
        img = np.expand_dims(img, axis=0)
        return img

    def main(self):
        while not rospy.is_shutdown():
            try:
                frame = self.frame
                frame_b = frame.copy()
                frame_r = frame.copy()
                #frame = cv.cvtColor(frame,cv.COLOR_RGB2BGR)
                img_hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
                #umbral de valores para el filtrado de cada color
                #red
                color_min_b=np.array([43,164,83])
                color_max_b=np.array([176,255,182])
                #color_min_r=np.array([0,80,106])  #manana
                #color_max_r=np.array([33,176,255])#manana
                color_min_r=np.array([140,61,135]) #tarde
                color_max_r=np.array([247,213,217])#tarde

                #Creamos las mascaras filtrando los colores
                mask_b=cv.inRange(img_hsv,color_min_b,color_max_b)
                mask_r=cv.inRange(img_hsv,color_min_r,color_max_r)
                #Filtarmos color correspondiente en cada frame
                #green filter
                frame_b[mask_b<255]=(0,0,0)
                frame_r[mask_r<255]=(0,0,0)
                #azul
                gray = cv.cvtColor(frame_b,cv.COLOR_BGR2GRAY)
                _,thresh = cv.threshold(gray,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
                kernel = np.ones((3,3),np.uint8)
                thresh = cv.dilate(thresh,kernel,iterations=5)
                contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
                #rojo
                gray_r = cv.cvtColor(frame_r,cv.COLOR_BGR2GRAY)
                _,thresh_r = cv.threshold(gray_r,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
                kernel = np.ones((3,3),np.uint8)
                thresh_r = cv.dilate(thresh_r,kernel,iterations=5)
                contours_r, hierarchy_r = cv.findContours(thresh_r, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
                #azul

                #rojo
                #print(len(contours_r))
                for cnt in contours:
                    area = cv.contourArea(cnt)
                    if area > 700:
                        x,y,w,h = cv.boundingRect(cnt)
                        cv.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                for cnt in contours_r:
                    area = cv.contourArea(cnt)
                    if area > 1000:
                        x,y,w,h = cv.boundingRect(cnt)
                        cv.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                img_back = self.bridge.cv2_to_imgmsg(frame)
                img_back.encoding = "bgr8"
                #print(img_back.encoding)
                self.pub_img.publish(img_back)

            except:
                print("vacio")

            self.rate.sleep()

if __name__ == "__main__":
    img = Imagen()
    img.main()
"""
                #color_min_b=np.array([59,165,84])
                #color_max_b=np.array([161,255,181])
                color_min_b=np.array([59,165,84])
                color_max_b=np.array([176,255,181])
                color_min_r=np.array([110,121,80])
                color_max_r=np.array([190,220,156])
"""
