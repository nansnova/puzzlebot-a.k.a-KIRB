#!/usr/bin/env python3
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from tensorflow.keras.models import load_model
import os, rospkg
class Imagen():
    def __init__(self):
        rospy.init_node("image_pub")
        rospy.Subscriber("/video_source/raw",Image,self.img_callback)
        self.frame = np.array([[]],dtype = "uint8")
        rp = rospkg.RosPack()
        self.script_path = os.path.join(rp.get_path("pista_manchester"), "src", "modelos","signals_5_student_final.model")
        self.model = load_model(self.script_path)
        self.classDictionary = {0: "stop", 1: "fin_prob", 2: "derecha", 3: "izquierda", 4: "siga", 5: "rotonda"}
        self.imageSize = (64, 64)
        self.rate = rospy.Rate(60)

    def img_callback(self,msg):
        #callback del punte entre ros y cv2
        self.frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

    def main(self):
        while not rospy.is_shutdown():
            try:
                frame = self.frame
                img = frame
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
                print(label,prob)
            except:
                print("vacio")

            self.rate.sleep()

if __name__ == "__main__":
    img = Imagen()
    img.main()
