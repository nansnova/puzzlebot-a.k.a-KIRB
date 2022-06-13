import cv2 as cv
import numpy as np


vc = cv.VideoCapture("nvarguscamerasrc ! nvvidconv ! video/x-raw, width=320, height=240, format=BGRx ! videoconvert !  video/x-raw, format=BGR ! appsink");

while True:
    next, frame = vc.read()
    frame_copy = frame.copy()
    #frame = cv.cvtColor(frame,cv.COLOR_RGB2BGR)
    frame2 = cv.resize(frame_copy,(320,280),interpolation=cv.INTER_NEAREST)
    gray_complete = cv.cvtColor(frame2,cv.COLOR_BGR2GRAY)
    gray_complete = cv.GaussianBlur(gray_complete,(7,7),1.5,1.5)
    rows = gray_complete.shape[0]
    can = cv.Canny(gray_complete, 100,30, 3)
    can_BGR = cv.cvtColor(can, cv.COLOR_GRAY2BGR)
    circles = cv.HoughCircles(can,cv.HOUGH_GRADIENT,dp = 1,minDist = 120,param1=60,param2=75,minRadius=10,maxRadius=120)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        thickness_center = 3
        thickness_outline = 3
        for i in circles[0,:]:
            center = (i[0],i[1])
            radius_outline = i[2]
            x,y,w,h = (center[0]-radius_outline-10,center[1]-radius_outline-10,radius_outline*2+20,radius_outline*2+20)
            #cv.rectangle(frame_copy,(x,y),(x+w,y+h),(0,255,0),2)
            cv.circle(frame_copy,center,radius_outline,(0,0,255),thickness_outline)
    cv.imshow("img",frame_copy)
    if cv.waitKey(50) >= 0:
        break;
