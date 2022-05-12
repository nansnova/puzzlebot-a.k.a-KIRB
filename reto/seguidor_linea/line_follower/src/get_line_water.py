#!/usr/bin/env python

import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

class getBlackLine():
    def __init__(self):
        rospy.init_node("get_black_line")
        rospy.Subscriber("/camera/image_raw",Image,self.img_callback)
        rospy.Subscriber("/wr",Float32,self.wr_callback)
        rospy.Subscriber("/wl",Float32,self.wl_callback)
        self.pub = rospy.Publisher("/img_line_res", Image, queue_size = 10)
        self.pub_error = rospy.Publisher("/err_line", Float32, queue_size = 10)
        self.pub_tiempo = rospy.Publisher("/tiempo", Float32, queue_size = 10)
        self.frame = np.array([[]],dtype = "uint8")
        self.wr = 0
        self.wl = 0
        self.bridge = CvBridge()
        self.rate = rospy.Rate(60)
    def img_callback(self,data):
        self.frame = self.bridge.imgmsg_to_cv2(data,desired_encoding = "passthrough")
    def wr_callback(self,data):
        self.wr = data.data
    def wl_callback(self,data):
        self.wl = data.data

    def main(self):
        while not rospy.is_shutdown():
            try:
                gray = cv.cvtColor(self.frame,cv.COLOR_BGR2GRAY)
                ret, thresh = cv.threshold(gray,0,255,cv.THRESH_BINARY_INV+cv.THRESH_OTSU)
                #Creamos el kernel
                kernel = np.ones((3,3),np.uint8)
                #Dilatamos la imagen para obtener el backgound
                sure_bg = cv.dilate(thresh,kernel,iterations=1)
                #Obtenemos el primer plano de la imagen
                dist_transform = cv.distanceTransform(thresh,cv.DIST_L2,5)
                ret, sure_fg = cv.threshold(dist_transform,0.01*dist_transform.max(),255,0)
                sure_fg = np.uint8(sure_fg)
                #Buscamos zonas deconocidas
                unknown = cv.subtract(sure_bg,sure_fg)
                #Obtenemos los marcadores de las zonas de colores
                ret, markers = cv.connectedComponents(sure_fg)
                #Evitamos marcadores con valores de 0
                markers = markers+1
                #Marcamos zonas desconocidas con 0
                markers[unknown==255] = 0
                #Ingresamoa la imagen y los marcadores en la funcion watersheed
                markers = cv.watershed(self.frame,markers)
                markers1 = markers.astype(np.uint8)
                ret, m2 = cv.threshold(markers1, 0, 255, cv.THRESH_OTSU)
                #markers.dtype = 'uint8'
                #markers = cv.resize(markers, (self.frame.shape[1],self.frame.shape[0]), interpolation = cv.INTER_NEAREST)
                #print(markers.shape)
                #markers= markers[440:,:]
                contours, _ = cv.findContours(m2,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
                img_back = cv.cvtColor(gray,cv.COLOR_GRAY2BGR)
                cv.drawContours(img_back, contours, -1, (0,255,0), 1)
                img_back = self.bridge.cv2_to_imgmsg(img_back)
                img_back.encoding = "bgr8"
                #print(img_back.encoding)
                self.pub.publish(img_back)
            except:
                print("vacio")
            self.rate.sleep()
if __name__ == "__main__":
    img = getBlackLine()
    img.main()
