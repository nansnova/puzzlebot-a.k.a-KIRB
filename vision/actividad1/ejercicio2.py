import cv2 as cv
from matplotlib import pyplot as plt
import numpy as np
import os

dir = "imgs_edificios/"
folder = os.listdir(dir)

for img in folder:
    imagen = cv.imread(dir+img)
    print(img)
    #Canny Edge Detection with threshold 50 and 200
    edges = cv.Canny(imagen,50,200)

    #Show results canny
    plt.subplot(121)
    plt.imshow(imagen)
    plt.title('Original Image')
    plt.xticks([]), plt.yticks([])
    plt.subplot(122)
    plt.imshow(edges,cmap='gray')
    plt.title('Edge Image')
    plt.xticks([]), plt.yticks([])
    plt.show()
    cv.waitKey()
    cv.destroyAllWindows()

    gray = cv.cvtColor(imagen, cv.COLOR_BGR2GRAY)
    #The Laplacian of an image
    laplacian = cv.Laplacian(gray,cv.CV_64F)
    #Sobelx Filter with a size 0f 5x5
    sobelx = cv.Sobel(gray,cv.CV_64F,1,0,ksize=5)
    #Sobelx Filter with a size 0f 5x5
    sobely = cv.Sobel(gray,cv.CV_64F,0,1,ksize=5)

    #Show results
    plt.subplot(2,2,1),plt.imshow(gray,cmap = 'gray')
    plt.title('Original'), plt.xticks([]), plt.yticks([])
    plt.subplot(2,2,2),plt.imshow(laplacian,cmap = 'gray')
    plt.title('Laplacian'), plt.xticks([]), plt.yticks([])
    plt.subplot(2,2,3),plt.imshow(sobelx,cmap = 'gray')
    plt.title('Sobel X'), plt.xticks([]), plt.yticks([])
    plt.subplot(2,2,4),plt.imshow(sobely,cmap = 'gray')
    plt.title('Sobel Y'), plt.xticks([]), plt.yticks([])
    plt.show()

    gray = cv.cvtColor(imagen, cv.COLOR_BGR2GRAY)
    edges = cv.Canny(gray,50,150,apertureSize = 3)
    lines = cv.HoughLinesP(edges,1,np.pi/180,100,minLineLength=100,maxLineGap=10)
    for line in lines:
        x1,y1,x2,y2 = line[0]
        cv.line(imagen,(x1,y1),(x2,y2),(0,255,0),2)
    cv.imwrite('houghlines.jpg',imagen)
    cv.imshow("Hough Lines in Road", imagen)

    imagen = cv.imread(dir+img)
    #Kernel Definition
    kernel = np.ones((5,5),np.uint8)
    #Erosion of an Image
    erosion = cv.erode(imagen,kernel,iterations = 1)
    #Dilation of an Image
    dilation = cv.dilate(imagen,kernel,iterations = 1)
    #Opening of an Image (Erosion followed by Dilation)
    opening = cv.morphologyEx(imagen, cv.MORPH_OPEN, kernel)
    #Closing of an Image (Dilation followed by Erosion)
    closing = cv.morphologyEx(imagen, cv.MORPH_CLOSE, kernel)
    #Morphological Gradient
    gradient = cv.morphologyEx(imagen, cv.MORPH_GRADIENT, kernel)
    #Top Hat Image (It is the difference between input image and Opening of the image)
    tophat = cv.morphologyEx(imagen, cv.MORPH_TOPHAT, kernel)
    #Black Hat (It is the difference between the closing of the input image and input image)
    blackhat = cv.morphologyEx(imagen, cv.MORPH_BLACKHAT, kernel)

    #Show results
    plt.subplot(2,4,1),plt.imshow(imagen,cmap = 'gray')
    plt.title("Original Binary Image"), plt.xticks([]), plt.yticks([])
    plt.subplot(2,4,2),plt.imshow(erosion,cmap = 'gray')
    plt.title("Eroded Image"), plt.xticks([]), plt.yticks([])
    plt.subplot(2,4,3),plt.imshow(dilation,cmap = 'gray')
    plt.title("Dilated Image"), plt.xticks([]), plt.yticks([])
    plt.subplot(2,4,4),plt.imshow(opening,cmap = 'gray')
    plt.title("Opened Image"), plt.xticks([]), plt.yticks([])
    plt.subplot(2,4,5),plt.imshow(closing,cmap = 'gray')
    plt.title("Closed Image"), plt.xticks([]), plt.yticks([])
    plt.subplot(2,4,6),plt.imshow(gradient,cmap = 'gray')
    plt.title("Morphological Gradient"), plt.xticks([]), plt.yticks([])
    plt.subplot(2,4,7),plt.imshow(tophat,cmap = 'gray')
    plt.title("Top Hat Image"), plt.xticks([]), plt.yticks([])
    plt.subplot(2,4,5),plt.imshow(blackhat,cmap = 'gray')
    plt.title("Black Hat Image"), plt.xticks([]), plt.yticks([])

    plt.show()

cv.waitKey()
cv.destroyAllWindows()
