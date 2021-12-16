#import rtde_control
import cv2
import numpy as np
import sys
"""
class arm_control(object):
    def __init__(self,ip):
        self.vel = 0.5
        self.acc = 0.3
        self.ip = ip
        self.is_moving = False
        try:
            self.rtde_c  = rtde_control.RTDEControlInterface(self.ip)
            print('Connected to UR3 , IP:' , self.ip)
        except:
            print(f"No connection. IP : {ip}")
            sys.exit()
    def move(self,point):
        self.is_moving = True
        print(f'Moving to {point} ...')
        self.rtde_c.moveL(point, self.vel, self.acc)
    def grab(self):
        pass
    def release(self):
        pass
    def map_coordinate(self):
        pass
    def calibrate(self):
        pass
    def reset(self):
        #self.move()
        self.is_moving = False
        pass
"""
#arm = arm_control(ip = "192.168.86.128")
#arm.move([-0.143, -0.435, 0.20, -0.001, 3.12, 0.04])

class detection(object):
    def __init__(self):
        self.cap = cv2.VideoCapture(0 ,  cv2.CAP_DSHOW)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        self.point = tuple()
        #self.arm = arm_control(ip = "192.168.86.128")
    def get_cnt(self,img , thr=200):
        contours,_ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        if len(contours) > 0:
            new_cnt = []
            for i in contours:
                if i.shape[0] > thr:
                    new_cnt.append(i)
            contours = tuple(new_cnt)
            return contours
        else:
            return tuple()

    def get_point(self,cnt):
        if len(cnt)> 0 :
            for c in cnt:
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            return cX,cY
        else:
            return tuple()

    def detect(self):
        while(1):
            ret , frame = self.cap.read()
            if ret :
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                
                red_mask = cv2.inRange(hsv, (0, 20, 20), (10, 255,255))
                green_mask = cv2.inRange(hsv, (45, 50, 50), (75, 255,255))
                blue_mask = cv2.inRange(hsv, (110, 20, 20), (130, 255,255))

                red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, self.kernel, iterations=2)
                green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, self.kernel, iterations=2)
                blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, self.kernel, iterations=2)

                red_cnt = self.get_cnt(red_mask)
                red_point = self.get_point(red_cnt)

                green_cnt = self.get_cnt(green_mask)
                green_point = self.get_point(green_cnt)

                blue_cnt = self.get_cnt(blue_mask)
                blue_point = self.get_point(blue_cnt)

                if self.arm.is_moving == False:
                    if len(red_point) > 0:
                        #self.arm.move()
                        #self.arm.grab()
                        #self.arm.move()
                        #self.arm.release()
                        #self.arm.reset()
                        pass

                    if len(green_point) > 0:
                        #self.arm.move()
                        #self.arm.grab()
                        #self.arm.move()
                        #self.arm.release()
                        #self.arm.reset()
                        pass

                    if len(blue_point) > 0:
                        #self.arm.move()
                        #self.arm.grab()
                        #self.arm.move()
                        #self.arm.release()
                        #self.arm.reset()
                        pass

                if len(red_point) > 0:
                    cv2.circle(frame, red_point, 10, (1, 227, 254), -1)
                if len(green_point) > 0:
                    cv2.circle(frame, green_point, 10, (1, 227, 254), -1)
                if len(blue_point) > 0:
                    cv2.circle(frame, blue_point, 10, (1, 227, 254), -1)

                cv2.drawContours(frame, red_cnt, -1, (0, 255, 0), 2)
                cv2.drawContours(frame, green_cnt, -1, (0, 255, 0), 2)
                cv2.drawContours(frame, blue_cnt, -1, (0, 255, 0), 2)

                cv2.imshow('src', frame)
                cv2.imshow('red' , red_mask)
                cv2.imshow('blue' , blue_mask)
                cv2.imshow('green' , green_mask)
            else:
                print('No camera connected')
            if cv2.waitKey(1) & 0xff == ord('q'):
                break
