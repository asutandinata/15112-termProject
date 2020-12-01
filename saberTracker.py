import cv2 as cv
import numpy as np
import math
import pygame
import time
pygame.init()
pygame.mixer.init()
cap = cv.VideoCapture(0)

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
    vecSquared=trueMag**2-(saberVector[0])**2-(saberVector[1])**2
    if(vecSquared<0):

        return 0
    else:
        return math.sqrt(vecSquared)

def get2DAngle(saberVector):
    v=np.array([1,0])
    angle=math.acos(np.dot(saberVector,v)/(np.linalg.norm(saberVector)*np.linalg.norm(v)))
    angle=math.degrees(angle)
    return (int(angle))

def get3DAngle(x,y):
    u=np.array([x,y])
    v=np.array([1,0])
    angle=math.acos(np.dot(u,v)/(np.linalg.norm(u)*np.linalg.norm(v)))
    angle=math.degrees(angle)
    return (angle)

#hMin,hMax,sMin,sMax,vMin,vMax=85,147,130,253,63,255
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
    cv.createTrackbar('sMax', 'sliders',255,255,nothing)
    cv.createTrackbar('vMax', 'sliders',255,255,nothing)
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
def stDev(data):
    mean=average(data)
    tempSum=0
    for val in data:
        tempSum+=(val-mean)**2
    return math.sqrt(tempSum/(len(data)-1))

def average(lst):
    return sum(lst)/len(lst)

def calibrateLength(minVal, maxVal):
    saberSizes=[]
    error=10
    font=cv.FONT_HERSHEY_SIMPLEX
    sizeCalibrated=False
    while(not sizeCalibrated):
        _, frame = cap.read()
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        hsv=cv.medianBlur(hsv,11)
        mask = cv.inRange(hsv, minVal, maxVal)
        kernel = np.ones((13,13),np.uint8)
        mask=cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        cv.putText(mask,f'now we will be calibrating saber length',(100,10), font, 0.5,(255,255,255),2,cv.LINE_AA)
        contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        cv.imshow('mask', mask)
        if len(contours)>0:# a contour is detected
            #contour detection
            rect = cv.minAreaRect(contours[0])
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.drawContours(mask,[box],0,(0,0,255),2)
            x0,y0=box[0]
            x1,y1=box[1]
            x2,y2=box[2]
            saberLength=max(distance(x0,y0,x1,y1),distance(x1,y1,x2,y2))
            saberSizes.append(saberLength)
        if(len(saberSizes)>25):
            dataError=stDev(saberSizes)
            if(dataError<error):
                sizeCalibrated=True
            else:
                saberSizes=[]
        
    cv.destroyWindow('mask')
    return average(saberSizes)

def getMidpoint(x0,y0,x1,y1):
    cx=(x0+x1)/2
    cy=(y0+y1)/2
    return(cx,cy)
def calibrateLighting():
    minVal,maxVal=calibrateSaberClick()
    calMin,calMax=calibrateSaberSlide(minVal,maxVal)
    lowerHSV=np.array(calMin)
    upperHSV=np.array(calMax)
    return lowerHSV, upperHSV

#general function, acts as 'main' for the cv loop
def generalTracking(lowerHSV, upperHSV, trueSaberLength):
    calibrated=False
    # lower_blue = np.array([62,91,82])
    # upper_blue = np.array([123,255,255])
    # trueSaberLength=calibrateLength(lower_blue,upper_blue)
    centers=[]
    XY=[]
    YZ=[]
    XZ=[]
    camWidth  = cap.get(3)
    camHeight = cap.get(4)
    cxPrev=0
    cyPrev=0
    while(True):
    
        startTime=time.time()
        _, frame = cap.read()
        # Convert BGR to HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        hsv=cv.medianBlur(hsv,11)

        #mask = cv.inRange(hsv, lower_blue, upper_blue)
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

            #determine the center of the saber
            x0,y0=box[0]
            x1,y1=box[1]
            x2,y2=box[2]
            x3,y3=box[3]
            minX=min(x0,x1,x2,x3)
            maxX=max(x0,x1,x2,x3)
            minY=min(y0,y1,y2,y3)
            maxY=max(y0,y1,y2,y3)
            cx,cy=getMidpoint(minX,minY,maxX,maxY)

            #determine length of saber
            saberLength1=distance(x0,y0,x1,y1)
            saberLength2=distance(x1,y1,x2,y2)
            if(saberLength1>saberLength2):
                saberLength=saberLength1
                saberWidth=saberLength2
                saberVector2D=np.array([x1-x0,y0-y1])
            else:
                saberLength=saberLength2
                saberWidth=saberLength1
                saberVector2D=np.array([x2-x1,y1-y2])

            #determine saber angles  

            centers.append((cx,cy,saberLength)) 
            xyAngle=get2DAngle(saberVector2D)
            z=getZVector(saberVector2D,trueSaberLength)
            saberVector=np.array([int(saberVector2D[0]),int(saberVector2D[1]),int(z)])#[x,y,z]
            yzAngle=get3DAngle(saberVector[1],saberVector[2])#direction vertically(swinging up and down)
            xzAngle=get3DAngle(saberVector[0],saberVector[2])#direction horizontally(swinging left and right)
            dt=time.time()-startTime
            XY.append(xyAngle)
            YZ.append(yzAngle)
            XZ.append(xzAngle)
            
            inSwing=isSwinging(centers)
            if inSwing:
                swing=getSwingDirection(XY,YZ,XZ,centers,dt)
                if swing!=None:
                    print(swing)
                    pass
            #debugging tools
            font=cv.FONT_HERSHEY_SIMPLEX
            #print(cx,cy)
            #print(f'pitch: {yzAngle}, yaw: {xzAngle}, plane angle: {xyAngle}')
            #cv.putText(res,f'dx: {cx-cxPrev}, dy: {cy-cyPrev}',(10,300), font, 1,(255,255,255),2,cv.LINE_AA)
            cv.putText(res,f'pitch: {yzAngle}, yaw: {xzAngle}, roll:{xyAngle}',(10,300), font, 1,(255,255,255),2,cv.LINE_AA)
            #cv.putText(res,f'{saberVector}',(10,300), font, 1,(255,255,255),2,cv.LINE_AA)
            cxPrev=cx
            cyPrev=cy
        
        cv.imshow('frame',frame) 
        cv.imshow('mask',mask)
        cv.imshow('result', res)

        k = cv.waitKey(5) & 0xFF
        if k == 27:
            break
    cv.destroyAllWindows()

def isSwinging(centers):
    minTime=5
    if(len(centers)<minTime):
        return False
    shortenedCenters=centers[-minTime:]
    for i in range(len(shortenedCenters)-2):
        x0,y0,len0=shortenedCenters[0]
        x1,y1,len1=shortenedCenters[1]
        #if saber stays same length or entire movement is parallel, then it's not a swing!
        if abs(len0-len1)<5 or distance(x0,y0,x1,y1)<10:
            return False
    #a swing is happening!
    return True

def getSwingDirection(XY,YZ,XZ,centers,dt):
    #THE CHANGE IN SABER CENTER INDICATES A SWING HAS BEGUN
    #THE ANGLES ARE THEN USED TO CHECK IF A SWING IS VALID
    #although it could all be done through looking at centers, analyzing the saber's angles will ensure a swing is proper and not just a translation

    #shorten to the most recent values for the swing
    minTime=3
    shortenedXY=XY[-minTime:]
    shortenedYZ=YZ[-minTime:]
    shortenedXZ=XZ[-minTime:]
    #dXYdt=(shortenedXY[minTime-1]-shortenedXY[minTime-2])/time
    dYZdt=abs((shortenedYZ[minTime-1]-shortenedYZ[minTime-2])/dt)
    dXZdt=abs((shortenedXZ[minTime-1]-shortenedXZ[minTime-2])/dt)

    xySpread=stDev(shortenedXY)
    yzSpread=stDev(shortenedYZ)
    xzSpread=stDev(shortenedXZ)
    x1,y1,l1=centers[-2]#second most recent
    x2,y2,l2=centers[-1]#most recent value
    dx=x2-x1
    dy=y2-y1
    dl=l2-l1
    XYangleError=8#maximum error an angle(XY) could have to consider the swing to be valid
    minMovement=10
    stDevError=20
    angleError=10
    
    #using each angle's derivative wrt time, and the standard deviations of the past few 'minTime' values, we find the swing direction

    if(xySpread<XYangleError) and (abs(dl)>10) and (dYZdt>5 or  dXZdt>5):
        #print(dx,dy)
        #print(yzSpread,xzSpread, dYZdt,dXZdt)
        if (dYZdt>10 and xzSpread<10) or (abs(dy)>15 and abs(dx)<12):
            if(dy<0):
                return'swinging up'
            else:
                return 'swinging down'
        elif (dXZdt>10 and dYZdt<10) or (abs(dx)>15 and abs(dy)<12):
            if dx>0:
                return 'swinging right'
            else:
                return 'swinging left'
        elif abs(abs(dx)-abs(dy))<30 and abs(dx)>6 and abs(dy)>6:
            if dx>0 and dy<0:
                return 'swinging up right'
            elif dx>0 and dy<0:
                return 'swinging up right'
            elif dx<0 and dy>0:
                return 'swinging down left'
            elif dx<0 and dy>0:
                return 'swinging down left'

    return None
    #types of swings:
    #swinging up or down,yz changes alot, xz doesnt change much, massive dyz/dt
    #swing left or right,yz doesnt change much, xz changes alot, large dxz/dt
    #swing Up right(NE), yz changes alot, xz changes alot
    #swing Up left(NW), 
    #swing down right(SE)
    #swing down left(SW)

    pass
    
#generalTracking()
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
