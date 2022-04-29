#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
class Imagen():
    def __init__(self):
        rospy.init_node("get_image")
        rospy.Subscriber("/video_source/raw",Image,self.img_callback)
        self.pub = rospy.Publisher("/imagen_ejer1", Image, queue_size = 10)
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        self.vel = Twist()
        self.frame = np.array([[]],dtype = "uint8")
        self.bridge = CvBridge()
        self.rate = rospy.Rate(60)
        self.h_g = 0

    def img_callback(self,data):
        frame = self.bridge.imgmsg_to_cv2(data,desired_encoding = "passthrough")
        self.frame = frame
    def track_callback(self,data):
        self.h_g = data

    def main(self):
        estado = "detenido"
        while not rospy.is_shutdown():
            #frame_rot = np.flip(self.frame,axis=0)
            #print(frame_rot)
            frame = self.frame
            frame_g = frame.copy()
            frame_r = frame.copy()
            frame_y = frame.copy()
            try:
                img_hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
                cv.createTrackbar("green call H", "cal", 0,255,self.track_callback)
                #print("h g: ",self.h_g)
                color_min_g=np.array([49,39,112])
                color_max_g=np.array([92,255,238])
                color_min_r=np.array([0,111,159])
                color_max_r=np.array([16,255,255])
                color_min_y=np.array([21,67,155])
                color_max_y=np.array([51,152,231])
                mask_g=cv.inRange(img_hsv,color_min_g,color_max_g)
                mask_r=cv.inRange(img_hsv,color_min_r,color_max_r)
                mask_y=cv.inRange(img_hsv,color_min_y,color_max_y)

                #print(mask)
                frame_g[mask_g<255]=(0,0,0)
                frame_r[mask_r<255]=(0,0,0)
                frame_y[mask_y<255]=(0,0,0)

                #green
                gray_g = cv.cvtColor(frame_g,cv.COLOR_BGR2GRAY)
                gray_g = cv.GaussianBlur(gray_g,(5,5),0)
                _, binary_g = cv.threshold(gray_g,0,255,cv.THRESH_BINARY_INV+cv.THRESH_OTSU)
                kernel = np.ones((3,3),np.uint8)
                erosion_g = cv.erode(binary_g,kernel,iterations = 2)
                dilation_g = cv.dilate(erosion_g,kernel,iterations = 2)
                size_g,pos_g = self.filter_circle(dilation_g)
                x_g,y_g = pos_g
                #print("x: ",x2,"y: ",y2,"size: ",round(size2))
                #print(x2,y2,round(size2))

                #red
                gray_r = cv.cvtColor(frame_r,cv.COLOR_BGR2GRAY)
                gray_r = cv.GaussianBlur(gray_r,(5,5),0)
                _, binary_r = cv.threshold(gray_r,0,255,cv.THRESH_BINARY_INV+cv.THRESH_OTSU)
                erosion_r = cv.erode(binary_r,kernel,iterations = 2)
                dilation_r = cv.dilate(erosion_r,kernel,iterations = 2)
                size_r,pos_r = self.filter_circle(dilation_r)
                x_r,y_r = pos_r
                #print(x3,y3,round(size3))

                #yellow
                gray_y = cv.cvtColor(frame_y,cv.COLOR_BGR2GRAY)
                gray_y = cv.GaussianBlur(gray_y,(5,5),0)
                _, binary_y = cv.threshold(gray_y,0,255,cv.THRESH_BINARY_INV+cv.THRESH_OTSU)
                erosion_y = cv.erode(binary_y,kernel,iterations = 2)
                dilation_y = cv.dilate(erosion_y,kernel,iterations = 2)
                size_y,pos_y = self.filter_circle(dilation_y)
                x_y,y_y = pos_y
                #print(x4,y4,round(size4))
                #cv.circle(frame_cir,(180,120),63,(255,0,0),-1)
                #print("verde: ",size_g,", rojo: ",size_r,", amarillo: ",size_y)
                if size_r > 0.0:
                    estado = "detenido"
                    self.vel.linear.x = 0.0
                    cv.circle(frame,(int(x_r),int(y_r)),int(size_r/2),(0,0,255),2)
                elif size_y > 0.0:
                    estado = "mitad vel"
                    self.vel.linear.x = 0.05
                    cv.circle(frame,(int(x_y),int(y_y)),int(size_y/2),(0,255,255),2)
                elif size_g > 0.0:
                    estado = "avanza"
                    self.vel.linear.x = 0.1
                    cv.circle(frame,(int(x_g),int(y_g)),int(size_g/2),(0,255,0),2)
                print("modo: ",estado)
                self.pub_vel.publish(self.vel)
                smaller =cv.resize(frame,(220,180),interpolation = cv.INTER_NEAREST)
                img_back = self.bridge.cv2_to_imgmsg(smaller)
                img_back.encoding = "bgr8"
                #print(img_back.encoding)
                self.pub.publish(img_back)
            except:
                print("vacio")
            self.rate.sleep()

    def filter_circle(self,bin_img):
        params = cv.SimpleBlobDetector_Params()

        # Change thresholds
        #params.minThreshold = 10;
        #params.maxThreshold = 20;

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 1000

        # Filter by Circularity
        params.filterByCircularity = True
        params.minCircularity = 0.7

        # Filter by Convexity
        params.filterByConvexity = False
        #params.minConvexity = 0.5

        # Filter by Inertia
        params.filterByInertia = False
        #params.minInertiaRatio = 0.9

        # Create a detector with the parameters
        #ver = (cv.__version__).split('.')
        #print(int(ver[0]))

        detector = cv.SimpleBlobDetector_create(params)
        keypoints = list(detector.detect(bin_img))

        try:
            tam = []
            pos = []
            for key in keypoints:
                tam.append(key.size)
                pos.append((key.pt[0],key.pt[1]))
            max_size = np.max(tam)
            index_max_tam = tam.index(max_size)
            x,y = pos[index_max_tam]
            features = (max_size,(x,y))
        except:
            features = (0,(0,0))
        return features

if __name__ == "__main__":
    img = Imagen()
    img.main()
