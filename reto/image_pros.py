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
        self.pub = rospy.Publisher("/imagen_ejer1", Image, queue_size = 1)
        self.frame = np.array([[]],dtype = "uint8")
        self.bridge = CvBridge()
        self.rate = rospy.Rate(60)
    def img_callback(self,data):
        frame =self.bridge.imgmsg_to_cv2(data,desired_encoding = "passthrough")
        self.frame = frame

    def main(self):
        while not rospy.is_shutdown():
            #frame_rot = np.flip(self.frame,axis=0)
            #print(frame_rot)
            try:
                gray = cv.cvtColor(self.frame,cv.COLOR_BGR2GRAY)
                gray_blur = cv.GaussianBlur(gray,(5,5),0)
                smaller = cv.cvtColor(cv.resize(gray_blur,(220,180),interpolation = cv.INTER_NEAREST),cv.COLOR_GRAY2BGR)
                img_back = self.bridge.cv2_to_imgmsg(smaller)
                img_back.encoding = "bgr8"
                #print(img_back.encoding)
                self.pub.publish(img_back)
            except:
                print("vacio")
            self.rate.sleep()

if __name__ == "__main__":
    img = Imagen()
    img.main()
