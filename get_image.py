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
        self.imagen = np.array([[]],dtype = "uint8")
        self.bridge = CvBridge()
        self.rate = rospy.Rate(1)
    def img_callback(self,data):
        frame =self.bridge.imgmsg_to_cv2(data,"bgr8")
        frame = np.array(frame,dtype=np.uint8)
        self.imagen = frame
        #self.imagen = cv_image
    def main(self):
        contador = 0
        while not rospy.is_shutdown():
            print(self.imagen.shape)
            if self.imagen.shape != (1,0):
                cv.imwrite("/home/elio987/Downloads/"+str(contador)+".png",self.imagen)
                #cv.destroyAllWindows()
                contador += 1
            self.rate.sleep()
if __name__ == "__main__":
    img = Imagen()
    img.main()
