import cv2 as cv
import numpy as np
#namedWindow("webcam")
vc = cv.VideoCapture(0);
"""
while True:
    next, frame = vc.read()
    imshow("webcam", frame)
    if waitKey(50) >= 0:
        break;
"""
while True:
    next, frame = vc.read()

    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    gauss = cv.GaussianBlur(gray, (7,7), 1.5, 1.5)
    can = cv.Canny(gauss, 0, 30, 3)
    gray_bor = cv.add(gauss,can)
    gray_bor_RGB = cv.cvtColor(gray_bor,cv.COLOR_GRAY2BGR)
    #print(np.max(gray_bor_RGB))
    border_color=np.array([255,255,255])

    # Mask image to only select browns
    mask=cv.inRange(gray_bor_RGB,border_color,border_color)

    # Change image to red where we found brown
    gray_bor_RGB[mask>0]=(211,2,209)
    cv.imshow("webcam", gray_bor_RGB)
    if cv.waitKey(50) >= 0:
        break;
