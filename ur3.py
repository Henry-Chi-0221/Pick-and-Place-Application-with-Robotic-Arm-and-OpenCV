import rtde_control
import cv2
import numpy as np
import sys
import platform
import rtde_io
import time
import threading
#192.168.1.100
class arm_control(object):
    def __init__(self,ip):
        self.vel = 0.6
        self.acc = 0.2
        self.ip = ip
        self.is_moving = False
        
        try:
            self.rtde_c  = rtde_control.RTDEControlInterface(self.ip)
            self.rtde_io = rtde_io.RTDEIOInterface(self.ip)
            print('Connected to UR3 , IP:' , self.ip)
        except:
            print(f"No connection. IP : {ip}")
            sys.exit()
        self.reset() #inital point
        self.rtde_io.setStandardDigitalOut(7, False)
    def move(self,point):
        self.is_moving = True
        #print(f'Moving to {point} ...')
        self.rtde_c.moveL(point, self.vel, self.acc)
    
    def grab(self , point , height):
        point[2] -= height
        self.move(point)
        time.sleep(0.5)
        self.rtde_io.setStandardDigitalOut(7, True)
        time.sleep(0.5)
        point[2] += height
        self.move(point)

    def release(self , point , height):
        point[2] -= height
        self.move(point)
        time.sleep(0.5)
        self.rtde_io.setStandardDigitalOut(7, False)
        time.sleep(0.5)
        point[2] += height
        self.move(point)

    def reset(self):
        self.move([0.29255 , -0.10271 , 0.33732 ,-2.224 , 2.218 , -0.032])
        self.is_moving = False
        

#arm = arm_control(ip = "192.168.86.128")
#arm.move([-0.143, -0.435, 0.20, -0.001, 3.12, 0.04])

class detection(object):
    def __init__(self):
        if platform.system() == "Windows":
            self.cap = cv2.VideoCapture(0 ,  cv2.CAP_DSHOW)
        else:
            self.cap = cv2.VideoCapture(0)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        self.point = tuple()
        #self.arm = arm_control(ip = "192.168.86.128")
        self.arm = arm_control(ip = "192.168.1.100")
        #self.k = 0.0011875
        self.k = 0.00119791666
        self.roi = [(320,0) , (480,0)]
        self.count_r = 1
        self.count_g = 1
        self.count_b = 1
    def get_cnt(self,img , thr=50):
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
    def to_red(self,point):
        point = list(point) 
        cam_x , cam_y = 320 , 240
        
        point[0] -= cam_x
        point[1] -= cam_y

        point[0]*=self.k #cam_x
        point[1]*=self.k #cam_y

        x , y = 0.370 , -0.015 # arm init
        y = y-point[0]
        x = x-point[1]
        self.arm.move([x,y,0.27188,-2.784,1.356,0.011])
        self.arm.grab([x,y,0.27188,-2.784,1.356,0.011] ,0.04808)

        self.arm.move([0.22832,-0.17379,0.27188,-2.784,1.356,0.011])
        self.arm.release([0.22832,-0.17379,0.27188,-2.784,1.356,0.011], 0.04808)
        self.arm.reset()
        
    def to_blue(self,point):
        point = list(point) 
        cam_x , cam_y = 320 , 240
        
        point[0] -= cam_x
        point[1] -= cam_y

        point[0]*=self.k #cam_x
        point[1]*=self.k #cam_y

        x , y = 0.370 , -0.015 # arm init
        y = y-point[0]
        x = x-point[1]
        self.arm.move([x,y,0.27188,-2.784,1.356,0.011])
        self.arm.grab([x,y,0.27188,-2.784,1.356,0.011] ,0.04808)

        self.arm.move([0.40833,-0.17379,0.27188,-2.784,1.356,0.011])
        self.arm.release([0.40833,-0.17379,0.27188,-2.784,1.356,0.011], 0.04808)
        self.arm.reset()
        
    def to_green(self,point):
        point = list(point) 
        cam_x , cam_y = 320 , 240
        
        point[0] -= cam_x
        point[1] -= cam_y

        point[0]*=self.k #cam_x
        point[1]*=self.k #cam_y

        x , y = 0.37613 , -0.0142 # arm init
        y = y-point[0]
        x = x-point[1]
        self.arm.move([x,y,0.27188,-2.784,1.356,0.011])
        self.arm.grab([x,y,0.27188,-2.784,1.356,0.011] ,0.04808)
        self.arm.reset()
        self.arm.move([0.40833 -0.17379,0.27188,-2.784,1.356,0.011])
        self.arm.release([0.40833 -0.17379,0.27188,-2.784,1.356,0.011], 0.04808)
        self.arm.reset()

    def calibration(self):
        while(1):
            ret,frame = self.cap.read()
            cv2.circle(frame, (320,240), 3, (0,0,255), -1)
            cv2.imshow('raw' , frame)
            if cv2.waitKey(1) & 0xff==ord('q'):
                break
    def detect(self):
        while(1):
            ret , frame = self.cap.read()
            if ret :
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                
                red_mask = cv2.inRange(hsv, (0, 20, 20), (10, 255,255))
                green_mask = cv2.inRange(hsv, (45, 50, 50), (75, 255,255))
                blue_mask = cv2.inRange(hsv, (100, 20, 20), (140, 255,255))

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
                        print('red')
                        
                        if red_point[0] <=self.roi[0][0] and red_point[0] >=self.roi[0][1] and red_point[1] <=self.roi[1][0] and red_point[1] >=self.roi[1][1] and  self.arm.is_moving == False:
                            threading.Thread(target = self.to_red ,args=(red_point,)).start()
                    """
                    if len(green_point) > 0 :

                        print('green')

                        if green_point[0] <=self.roi[0][0] and green_point[0] >=self.roi[0][1] and green_point[1] <=self.roi[1][0] and green_point[1] >=self.roi[1][1]:
                            threading.Thread(target = self.to_green ,args=(green_point,)).start()
                    """
                    if len(blue_point) > 0:
                        print("blue")
                        if blue_point[0] <=self.roi[0][0] and blue_point[0] >=self.roi[0][1] and blue_point[1] <=self.roi[1][0] and blue_point[1] >=self.roi[1][1] and  self.arm.is_moving == False:
                            threading.Thread(target = self.to_blue ,args=(blue_point,)).start()
                    
                if len(red_point) > 0:
                    #print(red_point)
                    cv2.circle(frame, red_point, 10, (1, 227, 254), -1)
                if len(green_point) > 0:
                    cv2.circle(frame, green_point, 10, (1, 227, 254), -1)
                if len(blue_point) > 0:
                    cv2.circle(frame, blue_point, 10, (1, 227, 254), -1)

                cv2.drawContours(frame, red_cnt, -1, (0, 255, 0), 2)
                #cv2.drawContours(frame, green_cnt, -1, (0, 255, 0), 2)
                cv2.drawContours(frame, blue_cnt, -1, (0, 255, 0), 2)
                cv2.rectangle(frame, (0, 0), (320, 480), (0,0,255), 3, cv2.LINE_AA)
                cv2.imshow('src', frame)
                cv2.imshow('red' , red_mask)
                cv2.imshow('blue' , blue_mask)
                #cv2.imshow('green' , green_mask)
            else:
                print('No camera connected')
            if cv2.waitKey(1) & 0xff == ord('q'):
                break
if __name__ == "__main__":
    d = detection()
    d.detect()
    #d.calibration()
