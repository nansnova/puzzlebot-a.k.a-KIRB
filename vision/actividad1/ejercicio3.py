# coding=utf-8

#Leonardo Gracida Munoz A01379812
#Nacy Lesly Gutierrez
import cv2 #opencv
import numpy as np
#Seleccionamos la camara a usar conectada por usb
vc = cv2.VideoCapture(0);
while True:
    #Obtenemos un frame
    next, frame = vc.read()
    #La cambiamos a escalas de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #Hacemos el espejo de la imagen
    gray = np.flip(gray, axis=1)
    frame = np.flip(frame, axis=1)
    #Cambiamos la escala de grises a tres canales BGR
    gray_BGR = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    #Agregamos un filtro Gussiano para eliminar ruido
    gauss = cv2.GaussianBlur(gray, (7,7), 1.5, 1.5)
    #Obtenemos los bordes de la imagen en escala de grises con el metodo Canny
    can = cv2.Canny(gauss, 0, 30, 3)
    #Obtenemos esta imagen con bordes a BGR
    can_BGR = cv2.cvtColor(can, cv2.COLOR_GRAY2BGR)
    #Cambiamos los colores de los bordes obtenidos
    #Declaramos el color del borde
    border_color=np.array([255,255,255])
    #Si el pixel es igual al color lo declaramos en la mascara
    mask=cv2.inRange(can_BGR,border_color,border_color)
    #Si la mascara dice que ese pixel era igual al del borde los cambiamos de color.
    can_BGR[mask>0]=(211,2,209)
    #Agregamos los bordes a la imagen en escala de grises
    gray_bor_RGB = cv2.add(gray_BGR,can_BGR)
    #Deteccion de polingonos
    #Binarizamos la imagen
    t, binary = cv2.threshold(gray,160, 255, cv2.THRESH_BINARY_INV)
    ################# Segmentación ########################
    contours,_ = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    ############Descripción y Extracción de características ############
    #Esto lo hacemos por cuantas figuras queramos reconocer dependiendo del numero de lados
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        ################ Reconocimiento ###########################
        #Triangulo
        if len(approx)==3:
            cv2.drawContours(gray_BGR,[cnt],0,(0,0,255),-1)
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        #Cuadrado
        if len(approx)==4:
            cv2.drawContours(gray_BGR,[cnt],0,(0,255,0),-1)
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        #Hexagono
        if len(approx)==5:
            cv2.drawContours(gray_BGR,[cnt],0,(255,0,0),-1)
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        #Heptagono
        if len(approx)==6:
            cv2.drawContours(gray_BGR,[cnt],0,(255,255,0),-1)
    #Deteccion de Circulos
    rows = gray.shape[0]
    #Obtenemos las coordenadas de los circulos con la sigueinte función
    circles = cv2.HoughCircles(can,cv2.HOUGH_GRADIENT,dp = 1,minDist = rows/8,
                            param1=80,param2=30,minRadius=25,maxRadius=40)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        radius_center = 1
        thickness_center = 3
        thickness_outline = 3
        for i in circles[0,:]:
            center = (i[0],i[1])
            #Dibujamos el centro del circulo
            cv2.circle(gray_BGR ,center,radius_center,(0,255,0),thickness_center)
            #Dibujamos el ciruclo
            radius_outline = i[2]
            cv2.circle(gray_BGR,center,radius_outline,(0,0,255),thickness_outline)
    #Reconocimeinto de lineas
    try:
        #Obtenemos las corrdenadas de las lineas detectadas
        lines = cv2.HoughLines(can,2,np.pi/180,200)
        for rho,theta in lines[0, :]:
            #Declaramos la posición y tamaño de la linea
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            cv2.line(gray_BGR,(x1,y1),(x2,y2),(0,0,255),2)
    except:
        print("lineas error")
    #Unimos las tres iamgenes en una sola
    final = np.concatenate((frame,gray_bor_RGB,gray_BGR),axis = 1)
    #Mostramos el resultado
    cv2.imshow("image",final)
    #cv2.waitKey()
    #cv2.destroyAllWindows()
    if cv2.waitKey(50) >= 0:
        break;
