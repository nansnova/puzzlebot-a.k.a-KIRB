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
	self.imagen = 0
        self.bridge = CvBridge()
        self.rate = rospy.Rate(60)
    def img_callback(self,data):
        frame =self.bridge.imgmsg_to_cv2(data,"bgr8")
        #frame_rot = np.flip(frame,axis=0)
	    #img_back = self.bridge.cv2_to_imgmsg(frame_rot,"bgr8")
        gray = cv.cvtColor(cv.cvtColor(frame,cv.COLOR_BGR2GRAY),cv.COLOR_GRAY2BGR)
        gray_blur = cv.GaussianBlur(gray,(5,5),0)
        smaller = cv.resize(gray_blur,(220,180),interpolation = cv.INTER_NEAREST)
        img_back = self.bridge.cv2_to_imgmsg(smaller,"bgr8")

	try:
	    self.pub.publish(img_back)
	    self.rate.sleep()
	except:
	    print(img_back)


    def main(self):
        rospy.spin()

if __name__ == "__main__":
    img = Imagen()
    img.main()
