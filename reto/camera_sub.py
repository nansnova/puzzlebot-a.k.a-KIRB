#!/usr/bin/env python
#Leoanrdo Gracida Munoz
#Nancy L. Garcia Jimenez

#Importamos las librerias de numpy cv2 los mensajes y el bridge de ROS msg a cv2
import rospy
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist

#Creamos la clase
class Imagen():
    def __init__(self):
        #Iniciamos el nodo
        rospy.init_node("get_image")
        #Nos sucribimos al topico de la imagen dada por la camara
        rospy.Subscriber("/video_source/raw",Image,self.img_callback)
        #Creamos el publicador al topico de la imagen resultante procesada
        self.pub = rospy.Publisher("/imagen_ejer1", Image, queue_size = 10)
        #Creamos el publicador al topico de comand velocity
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        #Iniciamos el mensaje de velocity
        self.vel = Twist()
        #Creamos la variable donde vamos a guardar la imagen obtenida de la camara
        self.frame = np.array([[]],dtype = "uint8")
        #Creamos el traductor de imagenes de ROS msg a cv2 y vicerversa
        self.bridge = CvBridge()
        #Declaramos los mensajes por segundo
        self.rate = rospy.Rate(60)
        #Declarmos que el robot va a detenerse al parar el codigo manualmente
        rospy.on_shutdown(self.end_callback)

    #funciones callback para extraer los datos de los suscriptores
    def img_callback(self,data):
        #callback del punte entre ros y cv2
        frame = self.bridge.imgmsg_to_cv2(data,desired_encoding = "passthrough")
        self.frame = frame

    #Callback de las acciones a hacer al cerrar el codigo manualmente
    def end_callback(self):
        self.vel.linear.x = 0
        self.pub_vel.publish(self.vel)

    def main(self):
        #Declaramos el estado inicial para mostrarlo en consola
        estado = "detenido"
        while not rospy.is_shutdown():
            #rotar el frame si axis=1
            frame = np.flip(self.frame,axis=0)
            frame = self.frame
            frame_g = frame.copy()
            frame_r = frame.copy()
            frame_y = frame.copy()
            try:
                #El frame obtenido lo pasamos a HSV
                img_hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
                #umbral de valores para el filtrado de cada color
                #red
                color_min_r=np.array([134,61,99])
                color_max_r=np.array([219,255,200])
                #green
                color_min_g=np.array([48,36,89])
                color_max_g=np.array([93,254,239])
                #yellow
                color_min_y=np.array([21,28,137])
                color_max_y=np.array([56,180,231])
                #Creamos las mascaras filtrando los colores
                mask_g=cv.inRange(img_hsv,color_min_g,color_max_g)
                mask_r=cv.inRange(img_hsv,color_min_r,color_max_r)
                mask_y=cv.inRange(img_hsv,color_min_y,color_max_y)

                #Filtarmos color correspondiente en cada frame
                #green filter
                frame_g[mask_g<255]=(0,0,0)
                #red filter
                frame_r[mask_r<255]=(0,0,0)
                #yellow filter
                frame_y[mask_y<255]=(0,0,0)

                #Por cada color vamos a hacer el mismo procesamiento.
                #green
                #Lo pasamos a escalas de grises la imagen filtrada solo del color verde del semaforo
                gray_g = cv.cvtColor(frame_g,cv.COLOR_BGR2GRAY)
                #Reducimos ruido
                gray_g = cv.GaussianBlur(gray_g,(5,5),0)
                #Se binariza con otsu
                _, binary_g = cv.threshold(gray_g,0,255,cv.THRESH_BINARY_INV+cv.THRESH_OTSU)
                kernel = np.ones((3,3),np.uint8)
                #erosion de la imagen
                erosion_g = cv.erode(binary_g,kernel,iterations = 2)
                #Dilatacion de la imagen
                dilation_g = cv.dilate(erosion_g,kernel,iterations = 2)
                #Usamos el Blob Detector para que nos devuelva el tamano y localizacion del circulo del semaforo
                size_g,pos_g = self.filter_circle(dilation_g)
                #Obtenemos la posicion
                x_g,y_g = pos_g

                #red
                #Lo pasamos a escalas de grises la imagen filtrada solo del color rojo del semaforo
                gray_r = cv.cvtColor(frame_r,cv.COLOR_BGR2GRAY)
                #Reduccion de ruido
                gray_r = cv.GaussianBlur(gray_r,(5,5),0)
                #binarizacion con otsu
                _, binary_r = cv.threshold(gray_r,0,255,cv.THRESH_BINARY_INV+cv.THRESH_OTSU)
                #Erosion de la imagen
                erosion_r = cv.erode(binary_r,kernel,iterations = 2)
                #Dilatacion de la imagen
                dilation_r = cv.dilate(erosion_r,kernel,iterations = 2)
                #Blob Detector para que nos devuelva el tamano y localizacion del circulo del semaforo
                size_r,pos_r = self.filter_circle(dilation_r)
                #Obtencion de la posicion
                x_r,y_r = pos_r
                #print(x3,y3,round(size3))

                #yellow
                #Lo pasamos a escalas de grises la imagen filtrada solo del color amarillo del semaforo
                gray_y = cv.cvtColor(frame_y,cv.COLOR_BGR2GRAY)
                gray_y = cv.GaussianBlur(gray_y,(5,5),0)
                _, binary_y = cv.threshold(gray_y,0,255,cv.THRESH_BINARY_INV+cv.THRESH_OTSU)
                erosion_y = cv.erode(binary_y,kernel,iterations = 2)
                dilation_y = cv.dilate(erosion_y,kernel,iterations = 2)
                size_y,pos_y = self.filter_circle(dilation_y)
                x_y,y_y = pos_y
                #print(x4,y4,round(size4))
                #print("verde: ",size_g,", rojo: ",size_r,", amarillo: ",size_y)

                """Si detecta un circulo de alguno de los colores correspondientes con un area mayor a veinte el robot
                se va a parar si es rojo, correr a mitad velocidad si es amarillo y avanzar si es verde.
                Las sigueintes condicionales permiten tener una jerarquia:
                Si el robot detecta rojo y alguno de los otros dos colores circulares son detectados,
                este se va a parar hasta que el rojo sea removido del frame, si detecta amarillo y verde,
                este se va a mover a mitad de velocidad hasta que el amarillo sea
                removido del frame, en caso de solo detectar verde se va a mover a velocidad normal y en caso de solo detectar amarillo
                se va a mover a mitad de velocidad."""
                if size_r > 20.0:
                    estado = "detenido"
                    self.vel.linear.x = 0.0
                    #En la posicion en la que lo detecta, dibuja un circulo rojo del tamaño detectado
                    cv.circle(frame,(int(x_r),int(y_r)),int(size_r/2),(0,0,255),2)
                    #Etiqueta el circulo que detecta con la leyenda "detenido"
                    cv.putText(frame, 'detenido', (int(10),int(50)), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 1, cv.LINE_AA)
                elif size_y > 20.0:
                    estado = "mitad vel"
                    self.vel.linear.x = 0.05
                    #En la posicion en la que lo detecta, dibuja un circulo amarillo del tamaño detectado
                    cv.circle(frame,(int(x_y),int(y_y)),int(size_y/2),(0,255,255),2)
                    #Etiqueta el circulo que detecta con la leyenda "mitad vel"
                    cv.putText(frame, 'mitad vel', (int(10),int(50)), cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 1, cv.LINE_AA)
                elif size_g > 20.0:
                    estado = "avanza"
                    self.vel.linear.x = 0.1
                    #En la posicion en la que lo detecta, dibuja un circulo verde del tamaño detectado
                    cv.circle(frame,(int(x_g),int(y_g)),int(size_g/2),(0,255,0),2)
                    #Etiqueta el circulo que detecta con la leyenda "avanza"
                    cv.putText(frame, 'avanza', (int(10),int(50)), cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 1, cv.LINE_AA)
                #print("modo: ",estado)
                #Publica en el topico /cmd_vel la velocidad
                self.pub_vel.publish(self.vel)
                #Establece el tamano de la ventana en 220x180
                smaller =cv.resize(frame,(220,180),interpolation = cv.INTER_NEAREST)
                #El tamano de la ventana anterior sera la usada como puente entre ros y cv2
                img_back = self.bridge.cv2_to_imgmsg(smaller)
                img_back.encoding = "bgr8"
                #print(img_back.encoding)
                self.pub.publish(img_back)
            except:
                print("vacio")
            self.rate.sleep()

    def filter_circle(self,bin_img):
        """Funcion que obteniendo un frame binarizado, detecta circulos, en base a las areas de pixeles negros conectados."""
        #Declaramos los parametros del Blob Detector
        params = cv.SimpleBlobDetector_Params()

        # Filtro por Area.
        params.filterByArea = True
        params.minArea = 1000

        # Filtro por Circularidad
        params.filterByCircularity = True
        params.minCircularity = 0.8

        # Filtro por Convexividad
        params.filterByConvexity = False

        # Filter por Inercia
        params.filterByInertia = False

        # Creacion del detector de circulos utilizando los parametros anteriores
        detector = cv.SimpleBlobDetector_create(params)
        # lista de valores proporcionada por el detctor
        keypoints = list(detector.detect(bin_img))

        try:
            tam = []
            pos = []
            #Iteramos entre todos los ciruclos detectados
            for key in keypoints:
                #Extraemos su tamano y su posicion
                tam.append(key.size)
                pos.append((key.pt[0],key.pt[1]))
            #Obtenemos la posicion y tamano del mas grande
            max_size = np.max(tam)
            index_max_tam = tam.index(max_size)
            x,y = pos[index_max_tam]
            #Lo retornamos
            features = (max_size,(x,y))
        except:
            #En caso de no encontrar nada devolver una tupla de ceros
            features = (0,(0,0))
        return features

if __name__ == "__main__":
    img = Imagen()
    img.main()
