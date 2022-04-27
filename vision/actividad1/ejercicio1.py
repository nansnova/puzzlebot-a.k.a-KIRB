#Leonardo Gracida Munoz A01379812
#Nacy Lesly Gutierrez

#Importamos las librerias necesarias a usar
import cv2 as cv
import os
import numpy as np
#Ponemos los dos archivos en los que vamos a guardar las imagenes
dir = "imagenes_road"
dir_img = "imagenes_res"
#Nombres de los archivos dentro de este folder
folder = os.listdir(dir)
#Variables del error gaussiano
mean = 0
sigma = 0.2
count = 0
#Hacemos el procesamiento por cada imagen
for img in folder:
    #Localidad de la imagen original
    path_img_or = os.path.join(dir, img)
    #Localidad del primer folder que vamos a crear
    path = os.path.join(dir_img, str(count))
    count += 1
    #Creamos el folder
    os.mkdir(path)
    #Abrimos la imagen
    imagen = cv.imread(path_img_or)
    #La cambiamos a escalas de grises
    gray = cv.cvtColor(imagen,cv.COLOR_BGR2GRAY)
    #Creamos el ruido Gaussiano basado en la imagen en escala de grises
    s = (np.random.normal(mean, sigma, (gray.shape[0],gray.shape[1]))).astype('uint8')
    #Binarizamos la imagen usando un treshhols
    th,im_th = cv.threshold(gray, 100, 255, cv.THRESH_BINARY)
    #Agregamos el ruido a la imagen en escala de grises
    gray_noise = cv.add(gray,s)
    #Aumentamos la desviacion estandar del error Gaussiano
    sigma += 0.1
    #Aqui guardamos las imegenes creadas
    img = img[:-4]
    path0 = os.path.join(path, img+str("_original.png"))
    cv.imwrite(path0,imagen)
    path1 = os.path.join(path, img+str("_gray.png"))
    cv.imwrite(path1,gray)
    path2 = os.path.join(path , img+str("_bin.png"))
    cv.imwrite(path2,im_th)
    path3 = os.path.join(path , img+str("_gray_noise.png"))
    cv.imwrite(path3,gray_noise)
    #Las mostramos
    cv.imshow("imagen color "+img,imagen)
    cv.imshow("imagen gray "+img,gray)
    cv.imshow("imagen bin "+img,im_th)
    cv.imshow("gray noise "+img,gray_noise)
    #Blur
    #Creamos las imagenes con blur
    avging = cv.blur(gray_noise,(10,10))
    cv.imshow("av_blur "+img,avging)
    gausBlur = cv.GaussianBlur(gray_noise,(5,5),0)
    cv.imshow("gaus blur "+img,gausBlur)
    medianBlur = cv.medianBlur(gray_noise, ksize=5)
    cv.imshow("median blur "+img,medianBlur)
    bilFilter = cv.bilateralFilter(gray_noise,9,75,75)
    cv.imshow("bilatereal fil "+img,bilFilter)
    #Guardamos las imagenes con blur
    path4 = os.path.join(path , img+str("_av_blur.png"))
    cv.imwrite(path4,avging)
    path4 = os.path.join(path , img+str("_av_blur.png"))
    cv.imwrite(path4,avging)
    path5 = os.path.join(path , img+str("_gauss_blur.png"))
    cv.imwrite(path5,gausBlur)
    path6 = os.path.join(path , img+str("_median_blur.png"))
    cv.imwrite(path6,medianBlur)
    path7 = os.path.join(path , img+str("_bilateral_fil.png"))
    cv.imwrite(path7,bilFilter)
    #Esperamos a apretar una tecla
    cv.waitKey()
    #Destruimos todas las ventanas
    cv.destroyAllWindows()
