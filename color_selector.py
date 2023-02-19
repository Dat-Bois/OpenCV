import cv2
import numpy as np
import argparse
import math
import time

#cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture("/Users/pc/Desktop/Code/OpenCV/gateSample_01.mov")

#cap = cv2.VideoCapture(r"C:\Users\Eesh\Documents\Robotics\OpenCV\gateSample_01.mov")


def drawBox(maskFrame, frameOG):
    font = cv2.FONT_HERSHEY_SIMPLEX
    cnts = cv2.findContours(maskFrame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    values = [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0), 0]
    #if len(cnts) > 0:
    for i in range(len(cnts)):
        #c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(cnts[i])
        c=cnts[i]
        # print(area)
        if(area > 300): #20
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            x2, y2, w, h = cv2.boundingRect(c)
            cv2.rectangle(frameOG, (x2, y2), (x2+w, y2+h), (0, 255, 0), 2)
            #cv2.circle(frameOG, (int(x), int(y)), int(radius), (255, 255, 255), 5, 2)
            values = [(x2, y2), (x2+w, y2), (x2, y2+h), (x2+w, y2+h), (int(x), int(y)), int(radius)]
            # print(values)
        else:
            values = [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0), 0]
    cv2.putText(frameOG, str(values), (10, 30), font, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imshow("Result", frameOG)


def on_trackbarHu(val):
    global upperH
    upperH = val


def on_trackbarSu(val):
    global upperS
    upperS = val


def on_trackbarVu(val):
    global upperV
    upperV = val


def on_trackbarHl(val):
    global lowerH
    lowerH = val


def on_trackbarSl(val):
    global lowerS
    lowerS = val


def on_trackbarVl(val):
    global lowerV
    lowerV = val


def get_color(frame):
    global upperH, upperS, upperV, lowerH, lowerS, lowerV
    upper = (upperH, upperS, upperV)
    lower = (lowerH, lowerS, lowerV)
    # print(upper)
    # print(lower)
    frame_new = cv2.GaussianBlur(frame, (5, 5), 0)
    # ------------------Brightness Normalization----------
    YUV = cv2.cvtColor(frame_new, cv2.COLOR_BGR2YUV)
    Y, U, V = cv2.split(YUV)
    frame_new2 = cv2.equalizeHist(Y)
    frame_new4 = cv2.merge((frame_new2, U, V))
    frame_new3 = cv2.cvtColor(frame_new4, cv2.COLOR_YUV2BGR)
    # --------------------
    hsv = cv2.cvtColor(frame_new3, cv2.COLOR_BGR2HSV)

    colorLower = lower
    colorUpper = upper

    mask = cv2.inRange(hsv, colorLower, colorUpper)
    #mask2 = cv2.inRange(hsv, colorLower2, colorUpper2)
    mask_final = mask  # + mask2
    #cv2.imshow("mask", mask_final)
    kernel = np.ones((3, 3), np.uint8)
    eroded = cv2.erode(mask_final, kernel, iterations=0)
    #dilated = cv2.dilate(mask_final, kernel, iterations=3)

    return eroded, frame_new3


global upperH, upperS, upperV, lowerH, lowerS, lowerV

on_trackbarHl(0)
on_trackbarSl(0)
on_trackbarVl(0)
on_trackbarHu(0)
on_trackbarSu(0)
on_trackbarVu(0)

cv2.namedWindow("HSV Picker")
cv2.createTrackbar("HueLower", "HSV Picker", 0, 360, on_trackbarHl)
cv2.createTrackbar("SatLower", "HSV Picker", 0, 255, on_trackbarSl)
cv2.createTrackbar("ValLower", "HSV Picker", 0, 255, on_trackbarVl)
cv2.createTrackbar("HueUpper", "HSV Picker", 0, 360, on_trackbarHu)
cv2.createTrackbar("SatUpper", "HSV Picker", 0, 255, on_trackbarSu)
cv2.createTrackbar("ValUpper", "HSV Picker", 0, 255, on_trackbarVu)

on_trackbarHl(0)
on_trackbarSl(64)
on_trackbarVl(0)
on_trackbarHu(17)
on_trackbarSu(226)
on_trackbarVu(222)


while (True):
    ret, frame = cap.read()
    if ret == True:
        cv2.imshow("OG", frame)
        cord = get_color(frame)
        drawBox(cord[0], cord[1])
        time.sleep(1/60)

    # -----------------
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
