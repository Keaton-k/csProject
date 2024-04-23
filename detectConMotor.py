import sys
import cv2
import numpy as np
from board import SCL,SDA
import busio
from adafruit_motor import sevo
from adafruit_pca9685 import PCA9685


#These are arbitrary channel values will proably have to change them
servo0 = servo.ContinousServo(pca.channels[2], min_pulse=900, max_pulse=2100)
servo1 = servo.ContinousServo(pca.channels[6], min_pulse=900, max_pulse=2100)

def driveForward():
    #temp
    servo0.throttle = 1.0
    servo1.throttle = 1.0


def driveBackwards():
    #temp
    servo0.throttle = -1.0
    servo1.throttle = -1.0


def turnRight():
    #temp
    servo0.throttle = 1.0
    servo1.throttle = 0.25


def turnLeft():
    #temp
    servo0.throttle = 0.25
    servo1.throttle = 1.0


def stop():
    servo0.throttle = 0.0
    servo1.throttle = 0.0

#color=input("what is the color \n")
#print(color,"\n")
#lighting=input("what is the lighting \n")


#cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(1)


def nothing(x) :
    pass

cv2.namedWindow("Trackbar")

if len(sys.argv)>=2:
    color = sys.argv[1]
    lighting=sys.argv[2]
else:
    color="r"
    lighting="r"

if color == "p" and lighting=="l":
    cv2.createTrackbar("L-H", "Trackbar", 118, 255,nothing)
    cv2.createTrackbar("L-S", "Trackbar", 69, 255,nothing)
    cv2.createTrackbar("L-V", "Trackbar",80, 180,nothing)
    cv2.createTrackbar("U-H", "Trackbar", 180, 180,nothing)
    cv2.createTrackbar("U-S", "Trackbar", 255, 255,nothing)
    cv2.createTrackbar("U-V", "Trackbar", 255, 255,nothing)
elif color == "p" and lighting=="d":
    cv2.createTrackbar("L-H", "Trackbar", 149, 255,nothing)
    cv2.createTrackbar("L-S", "Trackbar", 102, 255,nothing)
    cv2.createTrackbar("L-V", "Trackbar",16, 180,nothing)
    cv2.createTrackbar("U-H", "Trackbar", 180, 180,nothing)
    cv2.createTrackbar("U-S", "Trackbar", 255, 255,nothing)
    cv2.createTrackbar("U-V", "Trackbar", 255, 255,nothing)
else:
    cv2.createTrackbar("L-H", "Trackbar", 157, 255,nothing)
    cv2.createTrackbar("L-S", "Trackbar", 113, 255,nothing)
    cv2.createTrackbar("L-V", "Trackbar",180, 180,nothing)
    cv2.createTrackbar("U-H", "Trackbar", 180, 180,nothing)
    cv2.createTrackbar("U-S", "Trackbar", 255, 255,nothing)
    cv2.createTrackbar("U-V", "Trackbar", 255, 255,nothing)
    print("this color has not been implemented the default is red in dark lighting")




#This is the allows me to find other colors with trackbar
"""
cv2.createTrackbar("L-H", "Trackbar", 0, 255,nothing)
cv2.createTrackbar("L-S", "Trackbar", 0, 255,nothing)
cv2.createTrackbar("L-V", "Trackbar", 0, 180,nothing)
cv2.createTrackbar("U-H", "Trackbar", 180, 180,nothing)
cv2.createTrackbar("U-S", "Trackbar", 255, 255,nothing)
cv2.createTrackbar("U-V", "Trackbar", 255, 255,nothing)
"""

while True:
    _, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    l_h = cv2.getTrackbarPos("L-H", "Trackbar")
    l_s = cv2.getTrackbarPos("L-S", "Trackbar")
    l_v = cv2.getTrackbarPos("L-V", "Trackbar")
    u_h = cv2.getTrackbarPos("U-H", "Trackbar")
    u_s = cv2.getTrackbarPos("U-S", "Trackbar")
    u_v = cv2.getTrackbarPos("U-V", "Trackbar")



    lower_red = np.array([l_h,l_s,l_v])
    upper_red =  np.array([u_h,u_s,u_v])
   # lower_red = np.array([0,19,0])
   # upper_red =  np.array([180,255,255])

    mask = cv2.inRange(hsv, lower_red, upper_red)
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.erode(mask, kernel)

    # Contours/shape outlines
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    font=cv2.FONT_HERSHEY_COMPLEX

    for outline in contours:
        area = cv2.contourArea(outline)
        approx = cv2.approxPolyDP(outline, 0.05*cv2.arcLength(outline, True),True)
        x=approx.ravel()[0]
        y=approx.ravel()[1]
        # add if to limit noise like 400 pixel requirment to draw contour
        #if area > 400 code below
        if area > 500:
            cv2.drawContours(frame, [approx], 0, (0,0,0), 5)

            if len(approx) ==3:
                cv2.putText(frame,"triangle",(x,y),font,1,(0,0,0))
                driveForward();
                #print('its a triangle')
            elif len(approx) ==4:
                cv2.putText(frame,"rectangle",(x,y),font,1,(0,0,0))
                driveBackwards()
                #print('its a rectangle')
            elif len(approx) ==5:
                cv2.putText(frame,"pentagon",(x,y),font,1,(0,0,0))
                #print('its a pentagon')
                turnRight()
            elif len(approx) == 6:
                cv2.putText(frame,"hexagon",(x,y),font,1,(0,0,0))
                turnLeft()
            elif len(approx) >10 and len(approx) < 50:
                cv2.putText(frame,"circle",(x,y),font,1,(0,0,0))

       # else:
        #    print('its a circle')


    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()

servo0 = 0.0
serv1 = 0.0
pca.deinit()
