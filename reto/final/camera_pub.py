#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
class Imagen():
    def __init__(self):
        rospy.init_node("image_pub")
        rospy.Subscriber("/camera/image_raw",Image,self.img_callback)
        self.pub = rospy.Publisher("/video_source/raw", Image, queue_size = 10)
        self.bridge = CvBridge()
        self.frame = np.array([[]],dtype = "uint8")
        self.rate = rospy.Rate(60)
    def img_callback(self,data):
        #callback del punte entre ros y cv2
        self.frame = self.bridge.imgmsg_to_cv2(data,desired_encoding = "passthrough")
    def main(self):
        while not rospy.is_shutdown():
            try:
                frame = self.frame
                frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
                frame = cv.resize(frame,(320,280),interpolation = cv.INTER_NEAREST)
                img_back = self.bridge.cv2_to_imgmsg(frame)
                img_back.encoding = "bgr8"
                #print(img_back.encoding)
                self.pub.publish(img_back)
            except:
                print("vacio")

            self.rate.sleep()

if __name__ == "__main__":
    img = Imagen()
    img.main()
