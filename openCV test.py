import cv2 as cv
import numpy as np
import math
import pygame
pygame.init()
pygame.mixer.init()
cap = cv.VideoCapture(0)
soundEffect=pygame.mixer.Sound('Eating.wav')


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
    
#cv.namedWindow('slider')
hMin,hMax,sMin,sMax,vMin,vMax=85,147,130,253,63,255
# cv.createTrackbar('hMin', 'slider',0,179,nothing)
# cv.createTrackbar('sMin', 'slider',0,255,nothing)
# cv.createTrackbar('vMin', 'slider',0,255,nothing)
# cv.createTrackbar('hMax', 'slider',0,179,nothing)
# cv.createTrackbar('sMax', 'slider',0,255,nothing)
# cv.createTrackbar('vMax', 'slider',0,255,nothing)
#cv.createTrackbar('minLine', 'slider',0,255,nothing)
#cv.createTrackbar('blobArea', 'slider',0,2000,nothing)
while(True):
    _, frame = cap.read()
    # Convert BGR to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    #hsv=cv.GaussianBlur(hsv,(11,11),0)
    hsv=cv.medianBlur(hsv,11)
    lower_blue = np.array([84,125,37])
    upper_blue = np.array([107,255,255])
    # hMin = cv.getTrackbarPos('hMin','slider')
    # sMin = cv.getTrackbarPos('sMin','slider')
    # vMin = cv.getTrackbarPos('vMin','slider') 
    # hMax = cv.getTrackbarPos('hMax','slider')
    # sMax = cv.getTrackbarPos('sMax','slider')
    # vMax = cv.getTrackbarPos('vMax','slider')
    lower=np.array([hMin,sMin,vMin])
    upper=np.array([hMax,sMax,vMax])

    mask = cv.inRange(hsv, lower_blue, upper_blue)

    #denoise the mask, and dilate it a bit w/morphologiclal transforms https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_morphological_ops/py_morphological_ops.html
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
        #display the saber length onto the result
        if(angle==90 and not pygame.mixer.Channel(1).get_busy()):
            #pygame.mixer.Channel(1).play(soundEffect)0
            pass
        font=cv.FONT_HERSHEY_SIMPLEX
        cv.putText(res,f'length={saberLength},angle: {angle}',(10,300), font, 1,(255,255,255),2,cv.LINE_AA)
        
    else:
        print('saber not found')
    
    
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
