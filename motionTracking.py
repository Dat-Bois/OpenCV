import cv2
import numpy as np
import math
import time

ret = True
#cap = cv2.VideoCapture('sample.mp4')
cap  = cv2.VideoCapture(0)

width = 640
height = 480
edgeIgnore = 40
centerPoints_prev = []
trackingObjects = {}
track_id = 0
count=0
position = [width,height]

def processImage(frame, cnt):
    dim = (width, height)
    resized = cv2.resize(frame, dim)
    #contrast = cv2.convertScaleAbs(resized, alpha=1.5, beta=0)
    kernel = np.array([[0,-1,0],[-1,5,-1],[0,-1,0]])
    sharpened = cv2.filter2D(resized, -1, kernel)
    gray = cv2.cvtColor(sharpened, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,200,500)
    blur = cv2.GaussianBlur(edges,(5,5),0)
    ret, binary = cv2.threshold(blur, 10, 255, cv2.THRESH_BINARY)
    #
    #gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
    #   kernel = np.array([[0,-1,0],[-1,5,-1],[0,-1,0]])
    #   sharpened = cv2.filter2D(gray, -1, kernel)
    #contrast = cv2.convertScaleAbs(gray, alpha=1.5, beta=0)
    #blur = cv2.GaussianBlur(contrast,(5,5),0)
    ##   blur = cv2.bilateralFilter(contrast,9,75,75)
    #ret, binary = cv2.threshold(blur, 200, 255, cv2.THRESH_BINARY)
    #invert = cv2.bitwise_not(binary)
    #
    cv2.imshow('hsv', binary)
    #return binary
    return motionTrack(resized, binary, cnt) #invert

def motionTrack(frame, processed_frame, cnt):
    global centerPoints_prev
    global track_id
    global trackingObjects
    global position
    tempImg = np.zeros((height*2, width*2, 3), dtype = np.uint8)
    counter = 1
    sumX = sumY = 0
    centerPoints_curr = []
    contours, hierarchy = cv2.findContours(processed_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #TREE
    for cnts, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area>10 and area<6000):
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.intp(box)
                x, y, w, h = cv2.boundingRect(contour)
                cx = int((x+x+w)/2)
                cy = int((y+y+h)/2)
                if(not ((cx<edgeIgnore or cx>width-edgeIgnore) or (cy<edgeIgnore or cy>height-edgeIgnore))):
                    centerPoints_curr.append((cx,cy))
                cv2.drawContours(frame,[box],0,(255,0,0),2)
                #cv2.putText(frame, str(area), (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
    if(cnt<2):
        for pts in centerPoints_curr:
            for pts2 in centerPoints_prev:
                distance = math.hypot(pts2[0]-pts[0], pts2[1]-pts[1])
                if(distance<15):
                    trackingObjects[track_id] = pts
                    track_id+=1
                    if(track_id==5000):
                        track_id = 1
    else:
        centerPoints_curr_copy = centerPoints_curr.copy()
        trackingObjects_copy = trackingObjects.copy()
        for object_id, pts2 in trackingObjects_copy.items():
            objectExist = False
            for pts in centerPoints_curr_copy:
                   distance = math.hypot(pts2[0]-pts[0], pts2[1]-pts[1])
                   if(distance<10):
                      sumX += pts2[0]-pts[0]
                      sumY += pts2[1]-pts[1]
                      counter+=1
                      trackingObjects[object_id] = pts
                      objectExist = True
                      if pts in centerPoints_curr:
                        centerPoints_curr.remove(pts)
                      continue
            if not objectExist:
                trackingObjects.pop(object_id)

        for pts in centerPoints_curr:
            trackingObjects[track_id] = pts
            track_id+=1
            if(track_id==5000):
                track_id = 1

    for object_id, pt in trackingObjects.items():
         cv2.circle(frame, pt, 5, (0,0,255),-1)
         cv2.putText(frame, str(object_id), (pt[0], pt[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)

    position[0] = position[0] + int(sumX/counter)
    position[1] = position[1] + int(sumY/counter)
    cv2.circle(tempImg, position, 5, (0,0,255),-1)
    cv2.arrowedLine(tempImg, position, (width,height), (0,255,0), 3)
    cv2.imshow('pos', tempImg)
    centerPoints_prev = centerPoints_curr.copy()
    return frame

input("start")
while(True):
    ret, frame = cap.read()
    if(ret):
        try: 
            cv2.imshow('result', processImage(frame, count))
            time.sleep(1/30)
            count+=1
        except Exception as e:
            print(e)
            pass
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if cv2.waitKey(1) & 0xFF == ord('a'):
        centerPoints_prev = []
        trackingObjects = {}
        track_id = 0
        count=0
        position = [width,height]
cap.release()
cv2.destroyAllWindows()