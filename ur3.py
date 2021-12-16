import rtde_control
import cv2
import numpy as np
from time import time
ip = "192.168.86.128"
conn = False
try:
    rtde_c = rtde_control.RTDEControlInterface(ip)
    print('Connected to UR3 , IP:' , ip)
    conn = True
except:
    print(f"No connection. IP : {ip}")

if conn and 0:
    rtde_c.moveL([-0.143, -0.435, 0.20, -0.001, 3.12, 0.04], 3, 3)

def get_cnt(img , thr=200):
    contours,hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    if len(contours) > 0:
        new_cnt = []
        for i in contours:
            if i.shape[0] > thr:
                new_cnt.append(i)
        contours = tuple(new_cnt)
        return contours
    else:
        return tuple()


def get_point(cnt):
    if len(cnt)> 0 :
        for c in cnt:
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
        return cX,cY
    else:
        return tuple()
cap = cv2.VideoCapture(0 ,  cv2.CAP_DSHOW)

kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
while(1):
    s = time()
    ret , frame = cap.read()
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    red_mask = cv2.inRange(hsv, (0, 20, 20), (10, 255,255))
    green_mask = cv2.inRange(hsv, (45, 50, 50), (75, 255,255))
    blue_mask = cv2.inRange(hsv, (110, 20, 20), (130, 255,255))

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))

    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=2)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel, iterations=2)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel, iterations=2)

    red_cnt = get_cnt(red_mask)
    red_point = get_point(red_cnt)

    green_cnt = get_cnt(green_mask)
    green_point = get_point(green_cnt)

    blue_cnt = get_cnt(blue_mask)
    blue_point = get_point(blue_cnt)

    if len(red_point) > 0:
        cv2.circle(frame, red_point, 10, (1, 227, 254), -1)
    if len(green_point) > 0:
        cv2.circle(frame, green_point, 10, (1, 227, 254), -1)
    if len(blue_point) > 0:
        cv2.circle(frame, blue_point, 10, (1, 227, 254), -1)

    cv2.drawContours(frame, red_cnt, -1, (0, 255, 0), 2)
    cv2.drawContours(frame, green_cnt, -1, (0, 255, 0), 2)
    cv2.drawContours(frame, blue_cnt, -1, (0, 255, 0), 2)
    if ret :
        cv2.imshow('src', frame)
        cv2.imshow('red' , red_mask)
        cv2.imshow('blue' , blue_mask)
        cv2.imshow('green' , green_mask)
    else:
        print('No camera connected')
    if cv2.waitKey(1) & 0xff == ord('q'):
        break
    
