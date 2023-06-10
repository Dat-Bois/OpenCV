import cv2
import numpy as np
import math


ret = True
cap = cv2.VideoCapture('sample.mov')

width = 640
height = 480

def processImage(frame):
    dim = (width, height)
    resized = cv2.resize(frame, dim)
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    #kernel = np.array([[0,-1,0],[-1,5,-1],[0,-1,0]])
    #sharpened = cv2.filter2D(gray, -1, kernel)
    contrast = cv2.convertScaleAbs(gray, alpha=1.5, beta=0)
    blur = cv2.GaussianBlur(contrast,(5,5),0)
    #blur = cv2.bilateralFilter(contrast,9,75,75)
    ret, binary = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)
    invert = cv2.bitwise_not(binary)

    contours, hierarchy = cv2.findContours(invert, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #TREE
    for cnt, contour in enumerate(contours):
             area = cv2.contourArea(contour)
             rect = cv2.minAreaRect(contour)
             box = cv2.boxPoints(rect)
             box = np.intp(box)
             x, y, w, h = cv2.boundingRect(contour)
             if(area>10 and area<3000):
                cv2.drawContours(resized,[box],0,(255,0,0),2)
                cv2.putText(resized, str(area), (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
    cv2.imshow('binary', binary)
    cv2.imshow('contrast', contrast)
    return resized


while(ret):
    ret, frame = cap.read()
    try: 
        cv2.imshow('result', processImage(frame))
    except:
         pass
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()

#https://youtu.be/GgGro5IV-cs