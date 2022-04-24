# coding=utf-8
import cv2 #opencv
import numpy as np
vc = cv2.VideoCapture(0);
while True:
    ################## Adquisición de la Imagen ############
    #frame = cv2.imread("poligonos.jpg")
    next, frame = vc.read()
    ################# Procesamiento de la Imagen ##########
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = np.flip(gray, axis=1)
    gauss = cv2.GaussianBlur(gray, (7,7), 1.5, 1.5)
    can = cv2.Canny(gauss, 0, 30, 3)
    gray_bor = cv2.add(gauss,can)
    gray_bor_RGB = cv2.cvtColor(gray_bor,cv2.COLOR_GRAY2BGR)
    border_color=np.array([255,255,255])

    # Mask image to only select browns
    mask=cv2.inRange(gray_bor_RGB,border_color,border_color)

    # Change image to red where we found brown
    gray_bor_RGB[mask>0]=(211,2,209)
    gray_BGR = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    gray_BGR1 = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    gray_BGR2 = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    gray_BGR3 = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    gray_BGR4 = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    gray_BGR5 = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    #gauss = cv2.GaussianBlur(gray, (7,7), 1.5, 1.5)
    #scan = cv2.Canny(gauss, 0, 30, 3)
    t, binary = cv2.threshold(gray,160, 255, cv2.THRESH_BINARY_INV)
    ################# Segmentación ########################
    contours,_ = cv2.findContours(binary.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    ############Descripción y Extracción de características ############
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        ################ Reconocimiento ###########################
        if len(approx)==3:
            cv2.drawContours(gray_BGR1,[cnt],0,(0,0,255),-1)
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        if len(approx)==4:
            cv2.drawContours(gray_BGR2,[cnt],0,(0,255,0),-1)
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        if len(approx)==5:
            cv2.drawContours(gray_BGR3,[cnt],0,(255,0,0),-1)
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        if len(approx)==6:
            cv2.drawContours(gray_BGR4,[cnt],0,(255,255,0),-1)
    #Circulos
    rows = gray.shape[0]
    circles = cv2.HoughCircles(can,cv2.HOUGH_GRADIENT,dp = 1,minDist = rows/8,
                            param1=80,param2=30,minRadius=25,maxRadius=40)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        radius_center = 1
        thickness_center = 3
        thickness_outline = 3
        for i in circles[0,:]:
            center = (i[0],i[1])
            # draw the outer circle
            cv2.circle(gray_BGR5 ,center,radius_center,(0,255,0),thickness_center)
            # draw the center of the circle
            radius_outline = i[2]
            cv2.circle(gray_BGR5,center,radius_outline,(0,0,255),thickness_outline)
    gray_BGR_h1 = np.concatenate((gray_bor_RGB,gray_BGR1,gray_BGR2),axis = 1)
    gray_BGR_h2 = np.concatenate((gray_BGR3,gray_BGR4,gray_BGR5),axis = 1)
    gray_BGR = np.concatenate((gray_BGR_h1,gray_BGR_h2),axis = 0)
    scale_percent = 80 # percent of original size
    width = int(gray_BGR.shape[1] * scale_percent / 100)
    height = int(gray_BGR.shape[0] * scale_percent / 100)
    dim = (width, height)

    #Lineas
    # This returns an array of r and theta values
    try:
        lines = cv2.HoughLines(can,2,np.pi/180,200)
        for rho,theta in lines[0, :]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            cv2.line(gray_BGR5,(x1,y1),(x2,y2),(0,0,255),2)
    except:
        print("lineas error")

    # resize image
    gray_BGR = cv2.resize(gray_BGR5, dim, interpolation = cv2.INTER_AREA)
    #5cv2.imshow("image",frame)
    #cv2.imshow("image binary",binary)
    cv2.imshow("image",gray_BGR)
    #cv2.waitKey()
    #cv2.destroyAllWindows()
    if cv2.waitKey(50) >= 0:
        break;
