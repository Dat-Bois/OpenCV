import cv2
import numpy as np
import argparse
import math
import time

cap = cv2.VideoCapture(0)


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


global upperH, upperS, upperV, lowerH, lowerS, lowerV

upperH = upperS = upperV = 0
lowerH = lowerS = lowerV = 0

cv2.namedWindow("HSV Picker")
cv2.createTrackbar("HueLower", "HSV Picker", 0, 360, on_trackbarHl)
cv2.createTrackbar("SatLower", "HSV Picker", 0, 255, on_trackbarSl)
cv2.createTrackbar("ValLower", "HSV Picker", 0, 255, on_trackbarVl)
cv2.createTrackbar("HueUpper", "HSV Picker", 0, 360, on_trackbarHu)
cv2.createTrackbar("SatUpper", "HSV Picker", 0, 255, on_trackbarSu)
cv2.createTrackbar("ValUpper", "HSV Picker", 0, 255, on_trackbarVu)

cv2.setTrackbarPos("HueUpper", "HSV Picker", 360)
cv2.setTrackbarPos("SatUpper", "HSV Picker", 255)
cv2.setTrackbarPos("ValUpper", "HSV Picker", 255)


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
    cv2.imshow("mask", mask_final)
    kernel = np.ones((3, 3), np.uint8)
    eroded = cv2.erode(mask_final, kernel, iterations=0)

    return eroded, frame_new3


def calibrate(frame_init):
    cv2.imshow("Frame", frame_init)
    key = cv2.waitKey(1) & 0xFF
    if key == ord("s"):
        r = cv2.selectROI("Frame", frame_init,
                          fromCenter=False, showCrosshair=True)
        #frameCrop = frame_init[int(r[1]):int(r[1]+r[3]), int(r[0]):int(r[0]+r[2])]
        #width = int(r[3])
        #height = int(r[2])

    return r


def drawBox(maskFrame, frameOG):
    font = cv2.FONT_HERSHEY_SIMPLEX
    cnts = cv2.findContours(
        maskFrame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    values = [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0), 0]
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        area = cv2.contourArea(c)
        # print(area)
        if(area > 20):
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            x2, y2, w, h = cv2.boundingRect(c)
            cv2.rectangle(frameOG, (x2, y2), (x2+w, y2+h), (0, 255, 0), 2)
            cv2.circle(frameOG, (int(x), int(y)), int(
                radius), (255, 255, 255), 5, 2)
            values = [(x2, y2), (x2+w, y2), (x2, y2+h),
                      (x2+w, y2+h), (int(x), int(y)), int(radius)]
            # print(values)
        else:
            values = [(0, 0), (0, 0), (0, 0), (0, 0), (0, 0), 0]
    cv2.putText(frameOG, str(values), (10, 30), font,
                0.5, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imshow("Result", frameOG)


while (True):
    ret, frame = cap.read()
    if ret == True:
        cord = get_color(frame)
        #drawBox(cord[0], cord[1])
    # -----------------
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
