import cv2 as cv
import numpy as np
import math
cap = cv.VideoCapture(0)
def nothing(x):
    pass
    
cv.namedWindow('slider')
hMin,hMax,sMin,sMax,vMin,vMax=85,147,130,253,63,255
cv.createTrackbar('hMin', 'slider',84,179,nothing)
cv.createTrackbar('sMin', 'slider',125,255,nothing)
cv.createTrackbar('vMin', 'slider',37,255,nothing)
cv.createTrackbar('hMax', 'slider',107,179,nothing)
cv.createTrackbar('sMax', 'slider',250,255,nothing)
cv.createTrackbar('vMax', 'slider',250,255,nothing)

while(True):
    _, frame = cap.read()
    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    hsv=cv.medianBlur(hsv,11)
    # lower_blue = np.array([84,65,84])
    # upper_blue = np.array([107,255,255])
    hMin = cv.getTrackbarPos('hMin','slider')
    sMin = cv.getTrackbarPos('sMin','slider')
    vMin = cv.getTrackbarPos('vMin','slider') 
    hMax = cv.getTrackbarPos('hMax','slider')
    sMax = cv.getTrackbarPos('sMax','slider')
    vMax = cv.getTrackbarPos('vMax','slider')
    lower=np.array([hMin,sMin,vMin])
    upper=np.array([hMax,sMax,vMax])

    mask = cv.inRange(hsv, lower, upper)

    #denoise the mask, and dilate it a bit w/morphologiclal transforms https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
    kernel = np.ones((13,13),np.uint8)
    mask=cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    # mask=cv.dilate(mask,kernel, iterations=1)

    #flip each image vertically
    res = cv.bitwise_and(frame,frame, mask= mask)#overlay boolean mask onto frame
    frame=cv.flip(frame, 1)
    mask = cv.flip(mask, 1)
    res=cv.flip(res,1)
    
    cv.imshow('frame',frame) 
    cv.imshow('mask',mask)
    cv.imshow('result', res)

    k = cv.waitKey(5) & 0xFF
    if k == 27:
        break
cv.destroyAllWindows()
