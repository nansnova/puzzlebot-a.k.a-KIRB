#Leonardo Gracida Munoz A01379812
#Nacy Lesly Gutierrez
import cv2 as cv
import numpy as np
import os
import matplotlib.pyplot as plt
#Declaramos los directorios de la imagenes guardadas e imagenes a guardar
dir  = "senales/"
dir_guar_ruido = "imagenes_seg_res/ruido/"
dir_guar_normal = "imagenes_seg_res/normal/"
#Obtenemos los nombres de las imagenes
imagenes = os.listdir(dir)
#Valores del ruido Gaussiano
mean = 1
sigma = 1
count = 0
for img in imagenes:
    #Obtenemos el path de la imagen a usar
    imagen_path = dir+img
    #Obtenemos los datos de la imagen
    imagen = cv.imread(imagen_path)
    #Path para guarfar las imagenes sin ruido
    path_normal = os.path.join(dir_guar_normal, str(count))
    #Creamos el folder
    os.mkdir(path_normal)
    #Guardamos la imagen original
    cv.imwrite(os.path.join(path_normal, "imagen_normal.png"), imagen)
    #Pasamos la imagen a escalas de grises
    gray = cv.cvtColor(imagen,cv.COLOR_BGR2GRAY)
    #ELiminamos el ruido
    gray = cv.GaussianBlur(gray,(5,5),0.5)
    #Binarizacion normal
    th,im_th = cv.threshold(gray,125, 255, cv.THRESH_BINARY)
    #Guardamos la imagen en binarizacion normal
    cv.imwrite(os.path.join(path_normal, "imagen_bin.png"), im_th)
    #Binarizacion otsu
    th_ot,im_th_ot = cv.threshold(gray,0, 255, cv.THRESH_BINARY+cv.THRESH_OTSU)
    #Guardamos la imagen en binarizacion de otsu
    cv.imwrite(os.path.join(path_normal, "imagen_bin_ot.png"), im_th_ot)
    #Binarizacion triangle
    th_tr,im_th_tr = cv.threshold(gray,0, 255, cv.THRESH_BINARY+cv.THRESH_TRIANGLE)
    #Guardamos la imagen en binarizacion triangular
    cv.imwrite(os.path.join(path_normal, "imagen_bin_tri.png"), im_th_tr)
    #Complemento de la binarizacion.
    im_th = ~im_th
    im_th_ot = ~im_th_ot
    #Creamos el histograma de escalas de grises de la iamgen
    plt.figure()
    plt.hist(gray.ravel(),256,[0,256])
    #plt.show()

    #Watersheed
    #Binarizamos la imagen
    ret, thresh = cv.threshold(gray,0,255,cv.THRESH_BINARY_INV+cv.THRESH_OTSU)
    #Creamos el kernel
    kernel = np.ones((3,3),np.uint8)
    #Dilatamos la imagen para obtener el backgound
    sure_bg = cv.dilate(thresh,kernel,iterations=1)

    #Obtenemos el primer plano de la imagen
    dist_transform = cv.distanceTransform(thresh,cv.DIST_L2,5)
    ret, sure_fg = cv.threshold(dist_transform,0.01*dist_transform.max(),255,0)
    sure_fg = np.uint8(sure_fg)
    #Buscamos zonas deconocidas
    unknown = cv.subtract(sure_bg,sure_fg)
    #Obtenemos los marcadores de las zonas de colores
    ret, markers = cv.connectedComponents(sure_fg)
    #Evitamos marcadores con valores de 0
    markers = markers+1

    #Marcamos zonas desconocidas con 0
    markers[unknown==255] = 0
    #Ingresamoa la imagen y los marcadores en la funcion watersheed
    markers = cv.watershed(imagen,markers)
    markers.dtype = 'uint8'
    markers = cv.resize(markers, (imagen.shape[1],imagen.shape[0]), interpolation = cv.INTER_NEAREST)
    #Guardamos la imagen segmentada
    cv.imwrite(os.path.join(path_normal, "imagen_bin_mark.png"), markers)
    #cv.imshow("wat",markers)

    #Ruido
    #En esta zona hacemos lo mismo de antes pero con las iamgenes con ruido agregado
    path_ruido = os.path.join(dir_guar_ruido, str(count))
    os.mkdir(path_ruido)
    cv.imwrite(os.path.join(path_ruido, "imagen_normal.png"), imagen)

    #Creamos el ruido
    noise = np.random.normal(mean,sigma,imagen.size)
    noise = noise.reshape(imagen.shape[0],imagen.shape[1],imagen.shape[2]).astype('uint8')
    #Agregamos el ruido
    img_noise = cv.add(imagen,noise)
    gray_noise = cv.cvtColor(img_noise,cv.COLOR_BGR2GRAY)
    sigma += 0.1

    #Binarizacion normal
    th,im_th = cv.threshold(gray_noise,125, 255, cv.THRESH_BINARY)
    cv.imwrite(os.path.join(path_ruido, "imagen_bin.png"), im_th)
    #Binarizacion otsu
    th_ot,im_th_ot = cv.threshold(gray_noise,0, 255, cv.THRESH_BINARY+cv.THRESH_OTSU)
    cv.imwrite(os.path.join(path_ruido, "imagen_bin_ot.png"), im_th_ot)
    #Binarizacion triangulo
    th_tr,im_th_tr = cv.threshold(gray_noise,0, 255, cv.THRESH_BINARY+cv.THRESH_TRIANGLE)
    cv.imwrite(os.path.join(path_ruido, "imagen_bin_tri.png"), im_th_tr)
    #Complemento de la binarizacion.
    im_th = ~im_th
    im_th_ot = ~im_th_ot

    #Watersheed
    ret, thresh = cv.threshold(gray_noise,0,255,cv.THRESH_BINARY_INV+cv.THRESH_OTSU)
    kernel = np.ones((3,3),np.uint8)    # sure background area
    sure_bg = cv.dilate(thresh,kernel,iterations=1)
    dist_transform = cv.distanceTransform(thresh,cv.DIST_L2,5)
    ret, sure_fg = cv.threshold(dist_transform,0.01*dist_transform.max(),255,0)
    sure_fg = np.uint8(sure_fg)
    unknown = cv.subtract(sure_bg,sure_fg)
    ret, markers = cv.connectedComponents(sure_fg)
    markers = markers+1
    markers[unknown==255] = 0
    markers = cv.watershed(imagen,markers)
    markers.dtype = 'uint8'
    cv.imwrite(os.path.join(path_ruido, "imagen_bin_mark.png"), markers)
    count += 1
