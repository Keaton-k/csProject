import cv2
import numpy as np

cap = cv2.VideoCapture(0)

def nothing(x) :
    pass

cv2.namedWindow("Trackbar")

#This is the setup for the color red
cv2.createTrackbar("L-H", "Trackbar", 157, 255,nothing)
cv2.createTrackbar("L-S", "Trackbar", 113, 255,nothing)
cv2.createTrackbar("L-V", "Trackbar",180, 180,nothing)
cv2.createTrackbar("U-H", "Trackbar", 180, 180,nothing)
cv2.createTrackbar("U-S", "Trackbar", 255, 255,nothing)
cv2.createTrackbar("U-V", "Trackbar", 255, 255,nothing)



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

    for outline in contours:
        area = cv2.contourArea(outline)
        approx = cv2.approxPolyDP(outline, 0.05*cv2.arcLength(outline, True),True)
        # add if to limit noise like 400 pixel requirment to draw contour
        #if area > 400 code below
        if area > 50:
            cv2.drawContours(frame, [approx], 0, (0,0,0), 5)

            if len(approx) ==3:
                print('its a triangle')
            elif len(approx) ==4:
                print('its a rectangle')
            elif len(approx) ==5:
                print('its a pentagon')
       # else:
        #    print('its a circle')


    cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)

    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
