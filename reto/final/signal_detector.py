#!/usr/bin/env python
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from tensorflow.keras.models import load_model
import os
class Imagen():
    def __init__(self):
        rospy.init_node("signal_detector")
        rospy.Subscriber("/video_source/raw",Image,self.img_callback)
        self.pub = rospy.Publisher("/signals_detector", Image, queue_size = 10)
        self.bridge = CvBridge()
        self.frame = np.array([[]],dtype = "uint8")
        self.mainPath = "/home/elio987/Documents/deteccion_senales_CNN"
        self.modelPath = os.path.join(self.mainPath, "modelos")
        self.model = load_model(os.path.join(self.modelPath, "signals_5_student_final.model"))
        self.classDictionary = {0: "stop", 1: "fin_prob", 2: "derecha", 3: "izquierda", 4: "siga", 5: "rotonda"}
        self.imageSize = (64, 64)
        self.rate = rospy.Rate(60)
    def img_callback(self,data):
        #callback del punte entre ros y cv2
        self.frame = self.bridge.imgmsg_to_cv2(data,desired_encoding = "passthrough")
    def main(self):
        while not rospy.is_shutdown():
            try:
                frame = self.frame
                #frame = cv.cvtColor(frame,cv.COLOR_RGB2BGR)
                gray_complete = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
                rows = gray_complete.shape[0]
                can = cv.Canny(gray_complete, 150,30, 3)
                can_BGR = cv.cvtColor(can, cv.COLOR_GRAY2BGR)
                circles = cv.HoughCircles(can,cv.HOUGH_GRADIENT,dp = 1,minDist = rows/8,param1=80,param2=45,minRadius=20,maxRadius=120)
                if circles is not None:
                    circles = np.uint16(np.around(circles))
                    thickness_center = 3
                    thickness_outline = 3
                    for i in circles[0,:]:
                        center = (i[0],i[1])
                        radius_outline = i[2]
                        x,y,w,h = (center[0]-radius_outline-10,center[1]-radius_outline-10,radius_outline*2+20,radius_outline*2+20)
                        #cv.circle(frame,center,radius_outline,(0,0,255),thickness_outline)
                        #cv.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                        img = frame[y:y+h,x:x+w]
                        img = cv.cvtColor(img, cv.COLOR_BGR2RGB)
                        img = cv.resize(img, self.imageSize)
                        img = img.astype("float") / 255.0
                        img = np.expand_dims(img, axis=0)
                        predictions = self.model.predict(img)
                        #Obtenemos la maxima probabilidad
                        #cv.circle(frame,center,radius_outline,(0,0,255),thickness_outline)
                        classIndex = predictions.argmax(axis=1)[0]
                        #Obtenemos el label de la prediccion
                        label = self.classDictionary[classIndex]
                        prob = predictions[0][classIndex]
                        if prob > 0.9:
                            #print(label,prob)
                            cv.putText(frame, str(label)+": "+str(round(prob,2)), (x,y-15), cv.FONT_HERSHEY_SIMPLEX, 0.5,(0,255,0), 2)
                            cv.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)

                #frame = cv.resize(frame,(320,280),interpolation = cv.INTER_NEAREST)
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
    img.main()
