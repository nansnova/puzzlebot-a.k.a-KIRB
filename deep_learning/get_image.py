#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
class Imagen():
    def __init__(self):
        rospy.init_node("get_image")
        rospy.Subscriber("/video_source/raw",Image,self.img_callback)
        self.frame = np.array([[]],dtype = "uint8")
        self.bridge = CvBridge()
        self.rate = rospy.Rate(1)
        self.path = "/home/elio987/Documents/imagenes_deep_learning/"
    def img_callback(self,data):
        self.frame = self.bridge.imgmsg_to_cv2(data,desired_encoding = "passthrough")
        
        #self.imagen = cv_image
    def main(self):
        contador = 0
        while not rospy.is_shutdown():
            try:
                cv.imwrite(self.path+str(contador)+".png",self.frame)
                #cv.destroyAllWindows()
                contador += 1
            except:
                print("vacio")
            self.rate.sleep()
if __name__ == "__main__":
    img = Imagen()
    img.main()
