#!/usr/bin/env python
#Autores: Leonardo Gracida Munoz A0137, Nancy L.Garcia Jimenez A01378043
#Importamos las librerias de rospy, numpy y cv2
import rospy
import cv2 as cv
import numpy as np
#Importamos los mensjes necesarios
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
#Importamos el Bridge de cv2
from cv_bridge import CvBridge

class getBlackLine():
    def __init__(self):
        #Iniciamos el nodo
        rospy.init_node("get_black_line")
        #Creamos los subscribers necesarios
        rospy.Subscriber("/video_source/raw",Image,self.img_callback)
        rospy.Subscriber("/wr",Float32,self.wr_callback)
        rospy.Subscriber("/wl",Float32,self.wl_callback)
        #Creamos los Publishers
        self.pub = rospy.Publisher("/img_line_res", Image, queue_size = 10)
        self.pub_con_giro = rospy.Publisher("/con_giro", Float32, queue_size = 1)
        self.pub_error = rospy.Publisher("/err_line", Float32, queue_size = 10)
        #Variables usadas para obtener los datos de los subscribers
        self.frame = np.array([[]],dtype = "uint8")
        self.wr = 0
        self.wl = 0
        #Creamos el bridge de cv2 a ROS smg y viceversa
        self.bridge = CvBridge()
        #Mensajes por segundo
        self.rate = rospy.Rate(60)

    #Callbacks para obetener la informacion de los topicos
    def img_callback(self,data):
        self.frame = self.bridge.imgmsg_to_cv2(data,desired_encoding = "passthrough")
    def wr_callback(self,data):
        self.wr = data.data
    def wl_callback(self,data):
        self.wl = data.data
    def rec_img_cal(self,binaryImage):
        binaryImage = ~binaryImage
        # Get contours:
        contours, _ = cv.findContours(binaryImage, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        # Loop through contours:
        inPoints = np.zeros((4, 2), dtype="float32")
        for c in contours:

            # Approximate the contour to a polygon:
            perimeter = cv.arcLength(c, True)

            # Approximation accuracy:
            approxAccuracy = 0.05 * perimeter

            # Get vertices. Last flag indicates a closed curve:
            vertices = cv.approxPolyDP(c, approxAccuracy, True)

            # Print the polygon's vertices:
            verticesFound = len(vertices)
            #print("Polygon Vertices: " + str(verticesFound))

            # Prepare inPoints structure:

            # We have the four vertices that made up the
            # contour approximation:
            if verticesFound == 4:

                # Print the vertex structure:
                #print(vertices)

                # Format points:
                contador = 1
                for p in range(len(vertices)):
                    # Get corner points:
                    currentPoint = vertices[p][0]

                    # Store in inPoints array:
                    if (contador == 1):
                        limite = -250
                    elif (contador == 4):
                        limite = 250
                    elif (contador == 2):
                        limite = -260
                    elif (contador == 3):
                        limite = 260
                    contador += 1
                    inPoints[p][0] = currentPoint[0] + limite
                    inPoints[p][1] = currentPoint[1]

                    # Get x, y:
                    x = int(currentPoint[0] + limite)
                    y = int(currentPoint[1])
        targetWidth = 440
        targetHeight = targetWidth
        # Target Points:
        outPoints = np.array([
            [targetWidth, 0],  # 1
            [0, 0],  # 2
            [0, targetHeight],  # 3
            [targetWidth, targetHeight]],  # 4
            dtype="float32")
        return (inPoints,outPoints,targetWidth,targetHeight)

    def main(self):
        #Variables que necesitamos guardar
        estado = False
        #Puntos guardados de la imagen restificada
        inPoints,outPoints,tw,th = 0,0,0,0
        #Punto inicial al que alinearnos
        punto_0 = (0,0)
        #Variables booleanas para hacer el diferente control, deteccion de doble linea o giro muy cerrado.
        estado_giro = False
        estado_giro_fuerte = False
        while not rospy.is_shutdown():
            try:
                #Pasamos el frame a escalas de grises
                gray = cv.cvtColor(self.frame, cv.COLOR_BGR2GRAY)
                _, binaryImage = cv.threshold(gray, 75, 255, cv.THRESH_BINARY)
                #Kernel para hacer operaciones morfologicas
                kernel = np.ones((5,5),np.uint8)
    	        #erosionamos la imagen para eliminar ruido
    	        binaryImage = cv.erode(binaryImage,kernel,iterations = 3)
                #Cortamos la imagen a la mitad o mas de li mitad para no ver cosas inecesarias
                binaryImage = binaryImage[190:,:]
                #Obtenemos los puntos de la imagen rectificada una vez
                if estado == False:
                    inPoints,outPoints,tw,th = self.rec_img_cal(binaryImage)
                    estado = True
                #Cambiamos la perspectiva de la imagen para obtener en una imagen plana el camino. usando los puntos obtenidos
                H = cv.getPerspectiveTransform(inPoints, outPoints)
                #Cambiamos la perspectiva
                rectifiedImage = cv.warpPerspective(~binaryImage, H, (tw, th))
                rectifiedImage = np.flip(rectifiedImage.T,axis = 0)
                #Obtenemos el complementos
                rectifiedImage_com = ~rectifiedImage
                #Obtenemos los contornos
                contours, _ = cv.findContours(rectifiedImage,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
                x,y,w,h = cv.boundingRect(max(contours))
                punto_f = (int(x + w/2),int(y + h/2))
                if punto_0 == (0,0):
                    punto_0 = punto_f
                x1,y1 = punto_0
                x2,y2 = punto_f
                lim_izq = 0
                lim_der = rectifiedImage.shape[1]
                dis_ec = np.sqrt((x2-x1)**2 + (y2-y1)**2)
                dis_ec_x = np.sqrt((x2-x1)**2)
                dis_ec_y = np.sqrt((y2-y1)**2)
                dis_lim_der = np.sqrt((x2-lim_der)**2)
                dis_lim_izq = np.sqrt((x2-lim_izq)**2)
                if (dis_ec >= 300) and (estado_giro_fuerte == False):
                    estado_giro = True
                elif estado_giro == True:
                    giro = Float32()
                    giro.data = 0
                    self.pub_con_giro.publish(giro)
                    x,y = punto_0
                    punto_x = x
                    punto_y = y
                    if dis_ec_x < 10:
                        giro.data = 1
                        self.pub_con_giro.publish(giro)
                        estado_giro = False
                dis_lim = [dis_lim_izq,dis_lim_der]
                if min(dis_lim) == dis_lim_der:
                    lim = lim_der
                elif min(dis_lim) == dis_lim_izq:
                    lim = lim_izq
                if (min(dis_lim) <= 190) and (estado_giro == False):
                    estado_giro_fuerte = True
                if estado_giro_fuerte == True:
                    giro = Float32()
                    giro.data = 0
                    self.pub_con_giro.publish(giro)
                    punto_x = lim
                    punto_y = 0

                    if (dis_ec >= 300):
                        giro = Float32()
                        giro.data = 0
                        self.pub_con_giro.publish(giro)
                        estado_giro_fuerte = False
                        estado_giro = True
                        x,y = punto_0
                        punto_x = x
                        punto_y = y

                    if (min(dis_lim) > 80) and (estado_giro_fuerte == True):
                        giro.data = 1
                        self.pub_con_giro.publish(giro)
                        estado_giro_fuerte = False
                if (estado_giro == False)and(estado_giro_fuerte == False):
                    giro = Float32()
                    giro.data = 1
                    self.pub_con_giro.publish(giro)
                    x,y = punto_f
                    punto_x = x
                    punto_y = y
                    punto_0 = punto_f
                #Pintar
                img_back = cv.cvtColor(rectifiedImage_com, cv.COLOR_GRAY2BGR)
                cv.rectangle(img_back,(x-w,y-h),(x,y),(0,255,0),2)
                punto_x_des = int(rectifiedImage.shape[1]/2)
                error = punto_x_des - punto_x
                error_msg = Float32()
                error_msg.data = error
                self.pub_error.publish(error_msg)
                tiempo_msg = Float32()
                tiempo_msg.data = rospy.get_rostime().to_sec()
                self.pub_tiempo.publish(tiempo_msg)
                cv.circle(img_back,(punto_x,punto_y), 5, (0,0,255), -1)
                #print(error)
                w = 0.05*((self.wl-self.wr)/0.18)
                cv.putText(img_back,str(dis_ec),(10,30), cv.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,255),1,cv.LINE_AA)
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
