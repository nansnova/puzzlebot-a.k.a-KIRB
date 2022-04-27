import cv2 as cv
import numpy as np
import os
import matplotlib.pyplot as plt

dir  = "senales/"
imagenes = os.listdir(dir)
for img in imagenes:
    imagen_path = dir+img
    imagen = cv.imread(imagen_path)
    gray = cv.cvtColor(imagen,cv.COLOR_BGR2GRAY)
    gray = cv.GaussianBlur(gray,(5,5),0.5)
    histogram  = cv.calcHist(gray,[0],None,[256],[0,256])
    #Binarizacion normal
    th,im_th = cv.threshold(gray,125, 255, cv.THRESH_BINARY)
    #Binarizacion otsu
    th_ot,im_th_ot = cv.threshold(gray,0, 255, cv.THRESH_BINARY+cv.THRESH_OTSU)
    #Complemento de la binarizacion.
    im_th = ~im_th
    im_th_ot = ~im_th_ot
    #cv.imshow("img",imagen)
    #cv.imshow("gray",gray)
    plt.figure()
    plt.hist(gray.ravel(),256,[0,256])
    #plt.show()
    #cv.imshow("bin",im_th_ot)

    #Watersheed
    ret, thresh = cv.threshold(gray,0,255,cv.THRESH_BINARY_INV+cv.THRESH_OTSU)
    kernel = np.ones((3,3),np.uint8)    # sure background area
    sure_bg = cv.dilate(thresh,kernel,iterations=1)

    # Finding sure foreground area
    dist_transform = cv.distanceTransform(thresh,cv.DIST_L2,5)
    ret, sure_fg = cv.threshold(dist_transform,0.01*dist_transform.max(),255,0)
    # Finding unknown region
    sure_fg = np.uint8(sure_fg)
    unknown = cv.subtract(sure_bg,sure_fg)
    # Marker labelling
    ret, markers = cv.connectedComponents(sure_fg)
    # Add one to all labels so that sure background is not 0, but 1
    markers = markers+1

    # Now, mark the region of unknown with zero
    markers[unknown==255] = 0
    markers = cv.watershed(imagen,markers)
    imagen[markers == -1] = [255,0,0]
    markers.dtype = 'uint8'
    cv.imshow("wat",markers)
    cv.waitKey()
    cv.destroyAllWindows()
