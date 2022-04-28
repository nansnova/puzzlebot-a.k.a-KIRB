# coding=utf-8

import cv2 as cv #opencv
import numpy as np
#Seleccionamos la camara a usar conectada por usb
vc = cv.VideoCapture(0);
while True:
    next, frame = vc.read()
    frame = np.flip(frame, axis=1)
    img_hsv = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
    color_min=np.array([49,39,130])
    color_max=np.array([98,255,255])
    mask=cv.inRange(img_hsv,color_min,color_max)
    frame[mask<255]=(0,0,0)
    #print(np.max(mask))
    gray = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
    gray = cv.GaussianBlur(gray,(5,5),0)
    _, binary = cv.threshold(gray,0,255,cv.THRESH_BINARY_INV+cv.THRESH_OTSU)

    kernel = np.ones((3,3),np.uint8)
    erosion = cv.erode(binary,kernel,iterations = 3)
    dilation = cv.dilate(erosion,kernel,iterations = 3)
    dilation = ~dilation

    # Setup SimpleBlobDetector parameters.
    params = cv.SimpleBlobDetector_Params()

    # Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 200;

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 50

    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.85

    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.87

    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.01

    # Create a detector with the parameters
    ver = (cv.__version__).split('.')
    #print(int(ver[0]))

    detector = cv.SimpleBlobDetector_create(params)
    keypoints = list(detector.detect(dilation))
    dilation_BGR = cv.cvtColor(dilation,cv.COLOR_GRAY2BGR)
    print(dilation_BGR.shape)
    try:
        tam = []
        pos = []
        for key in keypoints:
            tam.append(key.size)
            pos.append((key.pt[0],key.pt[1]))
        max_size = np.max(tam)
        index_max_tam = tam.index(max_size)
        x,y = pos[index_max_tam]

    except:
        print("vacio")

    cv.imshow("filtro color",dilation_BGR)

    if cv.waitKey(50) >= 0:
        break;
