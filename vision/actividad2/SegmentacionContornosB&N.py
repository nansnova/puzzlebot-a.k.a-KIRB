from tkinter import *
from PIL import Image, ImageTk

import cv2
import numpy as np
import sys

def callback():
        ################## Adquisicion de la Imagen ############
        ret, frame = cam.read() # Leer Frame
        ####################### Mascara ########################
        kernel = np.ones((5,5),np.uint8) # Nucleo
        ################# Procesamiento de la Imagen ##########
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)# Convertir a BGR a grises
            t, binary = cv2.threshold(gray,umbralValue.get(),255, cv2.THRESH_BINARY)
            closing = cv2.morphologyEx(binary, cv2.MORPH_CLOSE,kernel)

            ################# Segmentacion de la Imagen ################
            contours,_ = cv2.findContours(closing.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            #print(len(contours))
            img_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
            h_min = cv2.getTrackbarPos("h_min","controls")
            h_max = cv2.getTrackbarPos("h_max","controls")
            s_min = cv2.getTrackbarPos("s_min","controls")
            s_max = cv2.getTrackbarPos("s_max","controls")
            v_min = cv2.getTrackbarPos("v_min","controls")
            v_max = cv2.getTrackbarPos("v_max","controls")

            color_min=np.array([h_min,s_min,v_min])
            color_max=np.array([h_max,s_max,v_max])
            mask=cv2.inRange(img_hsv,color_min,color_max)
            frame[mask<255]=(0,0,0)

            for cnt in contours:
                 cv2.drawContours (frame, [cnt], 0, (0,255,0), 3)

            # Mostrar imagen en el HMI
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(img)
            img.thumbnail((400,400))
            tkimage = ImageTk.PhotoImage(img)
            label.configure(image = tkimage )
            label.image = tkimage

            img1 = Image.fromarray(binary)
            img1.thumbnail((400,400))
            tkimage1 = ImageTk.PhotoImage(img1)
            label1.configure(image = tkimage1 )
            label1.image = tkimage1

            img2 = Image.fromarray(closing)
            img2.thumbnail((400,400))
            tkimage2 = ImageTk.PhotoImage(img2)
            label2.configure(image = tkimage2 )
            label2.image = tkimage2

            # Llamar a callback despues de 10 ms
            root.after(10,callback)

        else:
            Cerrar()

def thresholdVal(int):
    #Manipular el valor del slider
    umbralValue.set(slider1.get())

def Cerrar():
    print(f"Threshold value= {umbralValue.get()}")
    root.quit()                 #Salir del bucle
    cam.release()               #Cerrar camara
    print("Ip Cam Disconected")
    root.destroy()              #Destruye la ventana tkinter creada

########################### Cam ###########################
cam = cv2.VideoCapture(0)
if cam.isOpened():
    print("Ip Cam initializatized")
else:
    sys.exit("Ip Cam disconnected")

############################## HMI design #################
root = Tk()
root.protocol("WM_DELETE_WINDOW",Cerrar)
root.title("Vision Artificial") # titulo de la ventana

umbralValue = IntVar()

label=Label(root)
label.grid(row=0,padx=20,pady=20)

label1=Label(root)
label1.grid(row=0,column=1,padx=20,pady=20)

label2=Label(root)
label2.grid(row=1,column=0,padx=20,pady=20)

# Slider para recoger datos numericos
slider1 = Scale(root,label = 'Threshold value', from_=0, to=255, orient=HORIZONTAL,command=thresholdVal,length=400)
slider1.grid(row=1,column=1)

root.after(10,callback)
root.mainloop()
