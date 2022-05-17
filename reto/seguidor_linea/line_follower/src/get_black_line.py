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
        """Funcion que obtiene los puntos necesarios a usar para cambiar la perspectiva de una imagen y obtener
        la imagen de la linea a seguir de forma plana"""
        #Obtenemos el complemento de la imagen binarizada
        binaryImage = ~binaryImage
        #Obtenemos los contornos
        contours, _ = cv.findContours(binaryImage, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        #Matriz de los inPoints de las coordenadas de los cuato puntos del cacho de imagen que queremos cambiar la perspectiva
        inPoints = np.zeros((4, 2), dtype="float32")
        #Iteramos en todos los contronos detectados
        for c in contours:
            #Aproximamos un poligiono de los contornos obtenidos
            perimeter = cv.arcLength(c, True)
            #Aproximacion de exactitud
            approxAccuracy = 0.05 * perimeter
            #Obtenemos los vertices del poligino aproximado
            vertices = cv.approxPolyDP(c, approxAccuracy, True)
            #Cuantos vertices tiene
            verticesFound = len(vertices)
            #Si los vertices son cuatro significa que detectamso la linea
            if verticesFound == 4:
                contador = 1
                for p in range(len(vertices)):
                    #Obtenemos los puntos de las esquinas
                    currentPoint = vertices[p][0]
                    #En cada esquina la movemos un poco alejada del camino para tener mas vision
                    if (contador == 1):
                        limite = -250
                    elif (contador == 4):
                        limite = 250
                    elif (contador == 2):
                        limite = -260
                    elif (contador == 3):
                        limite = 260
                    contador += 1
                    #Obtenemos las coordenadas a usar para extraer de la imagen original el camino
                    inPoints[p][0] = currentPoint[0] + limite
                    inPoints[p][1] = currentPoint[1]
        #Declaramos el tamano de la imagen final resutlante del cambio de perspectiva
        targetWidth = 440
        targetHeight = targetWidth
        #Declaramos los outPoints para poder cambiar la perspectiva de la imagen
        outPoints = np.array([
            [targetWidth, 0],
            [0, 0],
            [0, targetHeight],
            [targetWidth, targetHeight]],
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
                #Obtenemos el complemento de la imagen rectificada binarizada
                rectifiedImage_com = ~rectifiedImage
                #Obtenemos los contornos de todo los detectado en la imagen
                contours, _ = cv.findContours(rectifiedImage,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
                #Obtenemos el rectagulo que encierre a la forma con contorno mas grande
                #En este caso el de la linea
                x,y,w,h = cv.boundingRect(max(contours))
                xr,yr,wr,hr = x,y,w,h
                #Guardamos las coordenadas del centro del rectagulo obtenido
                punto_f = (int(x + w/2),int(y + h/2))
                #En la primera iteracion el punto final e inicial son iguales
                if punto_0 == (0,0):
                    punto_0 = punto_f
                x1,y1 = punto_0
                x2,y2 = punto_f
                #Declaramos lo limites de la izquierda y derecha de la imagen rectificada
                lim_izq = 0
                lim_der = rectifiedImage.shape[1]
                #Obtenemos la distancia euclidianada entre el nuevo punto y el punto anterior
                dis_ec = np.sqrt((x2-x1)**2 + (y2-y1)**2)
                dis_ec_x = np.sqrt((x2-x1)**2)
                dis_ec_y = np.sqrt((y2-y1)**2)
                #Obtenemos las distancia euclidianadas entre el punto obtenido actual y los limites de izquierda y derecha
                dis_lim_der = np.sqrt((x2-lim_der)**2)
                dis_lim_izq = np.sqrt((x2-lim_izq)**2)
                """Para poder detectar dos lineas y e ir a la mas cercana lo que hicimos fue obtener la distancia euclidiana entre el
                punto anterior y el punto actual con el bounding rectangle, en caso de que haya una distancia
                euclidiana mayor a 300 y no estemos en un giro muy cerrado, guardamos el punto anterior que es el de la linea
                mas cercana, nos paramos en velocidad lineal y giramos hacia el punto anterior, hasta que el punto de la linea detectada
                mas grande tenga una diatancia euclidiana menor a 10."""
                if (dis_ec >= 300) and (estado_giro_fuerte == False):
                    estado_giro = True
                elif estado_giro == True:
                    #Nos paramos
                    giro = Float32()
                    giro.data = 0
                    self.pub_con_giro.publish(giro)
                    #Declaramos que los puntos a girar son los de la linea anterior
                    x,y = punto_0
                    punto_x = x
                    punto_y = y
                    #Si la distancia euclidiana es menor del nuevo punto detectado es menor a 10 podemos seguir avanzando
                    if dis_ec_x < 10:
                        giro.data = 1
                        self.pub_con_giro.publish(giro)
                        estado_giro = False
                """En caso de que haya una curva muy cerrada, lo que hacemos es obetener la distancia menor del punto a girar
                respeto al limite derecho o izquierdo, al tener esa distancia minima, si es menor a 190 y no estamos detectadno otra linea,
                vamos a parar y declara el giro hasta el limite de la imagen y vamos a parar hasta que la distancia entre el nuevo
                punto detectado y el limite sea mayor a 80"""
                #Obtenemos la distancia minima hacia el limite izquierdo o derecho
                dis_lim = [dis_lim_izq,dis_lim_der]
                if min(dis_lim) == dis_lim_der:
                    lim = lim_der
                elif min(dis_lim) == dis_lim_izq:
                    lim = lim_izq
                #Si la distancia es menor a 190
                if (min(dis_lim) <= 190) and (estado_giro == False):
                    estado_giro_fuerte = True
                if estado_giro_fuerte == True:
                    #Paramos la velocidad lineal
                    giro = Float32()
                    giro.data = 0
                    self.pub_con_giro.publish(giro)
                    #Declaramos ir hacia el limite izquierdo o derecho,
                    #Dependiendo de la distancia minima obtenida
                    punto_x = lim
                    punto_y = 0
                    #En caso de estar girando y detectar otra linea, lo que vamos a hacer es salir del control de giro cerrado
                    #Y vamos a entrar el control de giro con dos lineas detectadas
                    if (dis_ec >= 300):
                        #Paramos al robot en velocidad lineal
                        giro = Float32()
                        giro.data = 0
                        self.pub_con_giro.publish(giro)
                        #Salimos de esta condicional y entramos a la anterior
                        estado_giro_fuerte = False
                        estado_giro = True
                        x,y = punto_0
                        punto_x = x
                        punto_y = y
                    #En caso de ya tener una distancia mayor a 80 avanzamos otra vez hacia delante.
                    if (min(dis_lim) > 80) and (estado_giro_fuerte == True):
                        giro.data = 1
                        self.pub_con_giro.publish(giro)
                        estado_giro_fuerte = False
                #En caso de ir derecho o en curvas no cerradas actualizamos los puntos normalmente
                #Y vamos hacia delante
                if (estado_giro == False)and(estado_giro_fuerte == False):
                    giro = Float32()
                    giro.data = 1
                    self.pub_con_giro.publish(giro)
                    x,y = punto_f
                    punto_x = x
                    punto_y = y
                    punto_0 = punto_f
                #Al ya tener todos estos datos lo que hacemos es agregar componentes a la imagen final
                img_back = cv.cvtColor(rectifiedImage_com, cv.COLOR_GRAY2BGR)
                #Insetamos el bounding rectangle obtenido
                cv.rectangle(img_back,(xr,yr),(xr+wr,yr+hr),(0,255,0),2)
                #Declaramos el punto medio en x de la imagen como el punto a ir o girar para alinearse con la imagen
                punto_x_des = int(rectifiedImage.shape[1]/2)
                #Restamos con el punto obtenido en x del bounding rectangle para obtener el error
                error = punto_x_des - punto_x
                #Puzblicamos el error
                error_msg = Float32()
                error_msg.data = error
                self.pub_error.publish(error_msg)
                #Ponemos el punto medio del bounding rectangle
                cv.circle(img_back,(punto_x,punto_y), 5, (0,0,255), -1)
                #Obtenemos la velocidad angular del robot
                w = 0.05*((self.wl-self.wr)/0.18)
                #Mostramos la velocidadangular del robot
                cv.putText(img_back,str(w),(10,30), cv.FONT_HERSHEY_SIMPLEX, 0.5,(255,0,255),1,cv.LINE_AA)
                #Devolvemos la imagen resutlante a un nuevo topico
                img_back = self.bridge.cv2_to_imgmsg(img_back)
                img_back.encoding = "bgr8"
                self.pub.publish(img_back)
            except:
                #En caso de no recibir imagen imprimimos vacio
                print("vacio")
            #Aseguramos los Mensajes por segundo deseados
            self.rate.sleep()
if __name__ == "__main__":
    img = getBlackLine()
    img.main()
