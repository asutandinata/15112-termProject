import cv2
import numpy as np


cap = cv2.VideoCapture(0)

def nothing(x):
    pass
# Creating a window for later use
cv2.namedWindow('slider')

# Starting with 100's to prevent error while masking
h,s,v = 100,100,100

# Creating track bar
cv2.createTrackbar('h', 'slider',0,179,nothing)
cv2.createTrackbar('s', 'slider',0,255,nothing)
cv2.createTrackbar('v', 'slider',0,255,nothing)
cv2.createTrackbar('h1', 'slider',0,179,nothing)
cv2.createTrackbar('s1', 'slider',0,255,nothing)
cv2.createTrackbar('v1', 'slider',0,255,nothing)
while(1):

    _, frame = cap.read()

    #converting to HSV
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    # get info from track bar and appy to result
    h = cv2.getTrackbarPos('h','slider')
    s = cv2.getTrackbarPos('s','slider')
    v = cv2.getTrackbarPos('v','slider')
    h1 = cv2.getTrackbarPos('h1','slider')
    s1 = cv2.getTrackbarPos('s1','slider')
    v1 = cv2.getTrackbarPos('v1','slider')
    print(h,s,v)
    # Normal masking algorithm
    lower_blue = np.array([h,s,v])
    upper_blue = np.array([h1,s1,v1])
    mask = cv2.inRange(hsv,lower_blue, upper_blue)

    result = cv2.bitwise_and(frame,frame,mask = mask)

    cv2.imshow('result',mask)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cap.release()

cv2.destroyAllWindows()