import cv2 as cv
import numpy as np
import math
import pygame
pygame.init()
pygame.mixer.init()
cap = cv.VideoCapture(0)
saberMag=None
calibrated=False
H=[]
S=[]
V=[]
def nothing(x):
    pass
def distance(x0,y0,x1,y1):
    dx=x1-x0
    dy=y1-y0
    return int(math.sqrt(dx**2+dy**2))
def getZVector(saberVector, trueMag):
    return math.sqrt(trueMag**2-np.linalg.norm(saberVector**2))
def get2DAngle(saberVector):
    v=np.array([1,0])
    angle=math.acos(np.dot(saberVector,v)/(np.linalg.norm(saberVector)*np.linalg.norm(v)))
    angle=math.degrees(angle)
    return (int(angle))
    

hMin,hMax,sMin,sMax,vMin,vMax=85,147,130,253,63,255
def getValueOnClick(event,x,y,flags,hsv):
    if event == cv.EVENT_LBUTTONDOWN:
        print(y,x)
        HSVval=(hsv[y][x])
        H.append(HSVval[0])
        S.append(HSVval[1])
        V.append(HSVval[2])

def calibrateSaberClick():
    justClicked=False
    while len(H)<4:
        _, frame = cap.read()
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        hsv=cv.medianBlur(hsv,11)
        frame=cv.flip(frame, 1)
        hsv=cv.flip(hsv,1)
        cv.setMouseCallback('calibration screen',getValueOnClick,hsv)
        cv.imshow('calibration screen',hsv)
        if cv.waitKey(20) & 0xFF == 27:
            break

        if len(H)>4: 
            break   
    minVal=[min(H), min(S), min(V)]
    maxVal=[max(H),max(S),max(V)]
    cv.destroyWindow('calibration screen')
    return minVal,maxVal

def calibrateSaberSlide(defaultMin,defaultMax):
    calibrated=False
    cv.namedWindow('sliders')
    cv.createTrackbar('hMin', 'sliders',defaultMin[0],179,nothing)
    cv.createTrackbar('sMin', 'sliders',defaultMin[1],255,nothing)
    cv.createTrackbar('vMin', 'sliders',defaultMin[2],255,nothing)
    cv.createTrackbar('hMax', 'sliders',defaultMax[0],179,nothing)
    cv.createTrackbar('sMax', 'sliders',defaultMax[1],255,nothing)
    cv.createTrackbar('vMax', 'sliders',defaultMax[2],255,nothing)
    while(not calibrated):
        _, frame = cap.read()
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        hsv=cv.medianBlur(hsv,11)
        hMin = cv.getTrackbarPos('hMin','sliders')
        sMin = cv.getTrackbarPos('sMin','sliders')
        vMin = cv.getTrackbarPos('vMin','sliders') 
        hMax = cv.getTrackbarPos('hMax','sliders')
        sMax = cv.getTrackbarPos('sMax','sliders')
        vMax = cv.getTrackbarPos('vMax','sliders')
        lower=np.array([hMin,sMin,vMin])
        upper=np.array([hMax,sMax,vMax])

        mask = cv.inRange(hsv, lower, upper)
        kernel = np.ones((13,13),np.uint8)
        mask=cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        res = cv.bitwise_and(frame,frame, mask= mask)
        mask = cv.flip(mask, 1)
        cv.imshow('mask',mask)
        k = cv.waitKey(5) & 0xFF
        if k == 27:
            calibrated=True
    cv.destroyWindow('sliders')
    cv.destroyWindow('mask')
    
    return [hMin,sMin,vMin],[hMax,vMax,sMax]
def calibrateLength():
    
calibrated=False
while(True):
    if not calibrated:
        print('calibration started!')
        minVal,maxVal=calibrateSaberClick()
        calMin,calMax=calibrateSaberSlide(minVal,maxVal)
        lowerHSV=np.array(calMin)
        upperHSV=np.array(calMax)
        print(lowerHSV,upperHSV)
        calibrated=True

    _, frame = cap.read()
    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    hsv=cv.medianBlur(hsv,11)
    # lower_blue = np.array([84,125,37])
    # upper_blue = np.array([107,255,255])

    mask = cv.inRange(hsv, lowerHSV, upperHSV)
    
    kernel = np.ones((13,13),np.uint8)
    mask=cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    # mask=cv.dilate(mask,kernel, iterations=1)

    #flip each image vertically
    res = cv.bitwise_and(frame,frame, mask= mask)#overlay boolean mask onto frame
    frame=cv.flip(frame, 1)
    mask = cv.flip(mask, 1)
    res=cv.flip(res,1)

    #identify saber w/ contour detection
    
    contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    
    if len(contours)>0:
        #next 4 lines from here:# https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html
        rect = cv.minAreaRect(contours[0])
        box = cv.boxPoints(rect)
        box = np.int0(box)
        cv.drawContours(res,[box],0,(0,0,255),2)

        x0,y0=box[0]
        x1,y1=box[1]
        x2,y2=box[2]
        saberLength1=distance(x0,y0,x1,y1)
        saberLength2=distance(x1,y1,x2,y2)
        if(saberLength1>saberLength2):
            saberLength=saberLength1
            saberWidth=saberLength2
            saberVector=np.array([x1-x0,y1-y0])
            angle=get2DAngle(saberVector)
        else:
            saberLength=saberLength2
            saberWidth=saberLength1
            saberVector=np.array([x2-x1,y2-y1])
            angle=get2DAngle(saberVector)
        

        font=cv.FONT_HERSHEY_SIMPLEX
        cv.putText(res,f'length={saberLength},angle: {angle}',(10,300), font, 1,(255,255,255),2,cv.LINE_AA)
        
    else:
        pass
        #print('saber not found')
    
    cv.imshow('frame',frame) 
    cv.imshow('mask',mask)
    cv.imshow('result', res)

    k = cv.waitKey(5) & 0xFF
    if k == 27:
        break
cv.destroyAllWindows()


    

#previously used code for figuring out best way to mask

    #do some blob detection based on the mask, minimum area
    # blobParam=cv.SimpleBlobDetector_Params()
    # blobParam.filterByArea=True
    # blobArea = cv.getTrackbarPos('blobArea','slider')
    # blobParam.minArea=blobArea
    # detector=cv.SimpleBlobDetector_create(blobParam)
    # blobs=detector.detect(mask)
    # blobImage = cv.drawKeypoints(mask, blobs, np.array([]), (0,0,255), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

#harris corner detection
    #harris corner detection onto result image
    # greyRes=cv.cvtColor(res,cv.COLOR_BGR2GRAY)
    # greyRes=np.float32(greyRes)
    # dst = cv.cornerHarris(mask,4,3,0.01)#https://docs.opencv.org/master/dd/d1a/group__imgproc__feature.html#gac1fc3598018010880e370e2f709b4345
    # dst = cv.dilate(dst,None)
    # res[dst>0.01*dst.max()]=[0,0,0]

#edge detection
    #edges=cv.Canny(mask,255,255)
