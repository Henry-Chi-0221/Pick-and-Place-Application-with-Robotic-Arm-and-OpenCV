import rtde_control
import cv2

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

cap = cv2.VideoCapture('')
ret , frame = cap.read()
if ret :
    cv2.imshow('src', frame)
else:
    print('No camera connected')
