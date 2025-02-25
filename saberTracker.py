import cv2 as cv
import numpy as np
import math
import pygame
import time
import random
#initialize pygame, and the camera
pygame.init()
pygame.mixer.init()
cap = cv.VideoCapture(0)
cap.set(3,1280)#width
cap.set(4,720)#height

#soundfiles taken straight from actual gamefiles of beatsaber
hit=pygame.mixer.Sound('hit.wav')
missed=pygame.mixer.Sound('MissedNote.wav')
calibrated=False
H=[]
S=[]
V=[]

def nothing(x):
    pass

#distance formula
def distance(x0,y0,x1,y1):
    dx=x1-x0
    dy=y1-y0
    return int(math.sqrt(dx**2+dy**2))

#returns how much saber is tilting forward based on it's calculated length
def getZVector(saberVector, trueMag):
    vecSquared=trueMag**2-(saberVector[0])**2-(saberVector[1])**2
    if(vecSquared<0):
        return 0
    else:
        return math.sqrt(vecSquared)

#returns angle based on saber positioning
def get2DAngle(saberVector):
    v=np.array([1,0])
    angle=math.acos(np.dot(saberVector,v)/(np.linalg.norm(saberVector)*np.linalg.norm(v)))
    angle=math.degrees(angle)
    return (int(angle))

#returns angle given an x or y and z vector
def get3DAngle(x,y):
    u=np.array([x,y])
    v=np.array([1,0])
    angle=math.acos(np.dot(u,v)/(np.linalg.norm(u)*np.linalg.norm(v)))
    angle=math.degrees(angle)
    return (angle)

#helper function to detect clicks
def getValueOnClick(event,x,y,flags,hsv):
    if event == cv.EVENT_LBUTTONDOWN:
        HSVval=(hsv[y][x])
        H.append(HSVval[0])
        S.append(HSVval[1])
        V.append(HSVval[2])

#rough calibration window by just clicking on object
def calibrateSaberClick():
    justClicked=False
    while len(H)<4:
        _, frame = cap.read()
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        hsv=cv.medianBlur(hsv,11)
        frame=cv.flip(frame, 1)
        hsv=cv.flip(hsv,1)
        
        cv.setMouseCallback('calibration screen',getValueOnClick,hsv)
        
        font=cv.FONT_HERSHEY_SIMPLEX
        
        cv.putText(hsv, f'Click on the saber {4-len(H)} more times',(0,30),font, 1,(255,255,255),2,cv.LINE_AA)
        cv.imshow('calibration screen',hsv)
        if cv.waitKey(20) & 0xFF == 27:#waitkey usage from https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html
            break
        if len(H)>4: 
            break   
    minVal=[min(H), min(S), min(V)]
    maxVal=[max(H),max(S),max(V)]
    cv.destroyWindow('calibration screen')
    return minVal,maxVal

#slider window for fine calibration
def calibrateSaberSlide(defaultMin,defaultMax):
    calibrated=False
    cv.namedWindow('sliders')
    cv.createTrackbar('hMin', 'sliders',defaultMin[0],179,nothing)
    cv.createTrackbar('sMin', 'sliders',defaultMin[1],255,nothing)
    cv.createTrackbar('vMin', 'sliders',defaultMin[2],255,nothing)
    cv.createTrackbar('hMax', 'sliders',defaultMax[0]+5,179,nothing)
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
        #next 2 lines based on guide on morphological transform: https://docs.opencv.org/master/d9/d61/tutorial_py_morphological_ops.html
        kernel = np.ones((13,13),np.uint8)
        mask=cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        res = cv.bitwise_and(frame,frame, mask= mask)
        mask = cv.flip(mask, 1)
        font=cv.FONT_HERSHEY_SIMPLEX
        cv.putText(mask, 'adjust sliders until only saber is consistently visible',(0,30),font, 1,(255,255,255),2,cv.LINE_AA)
        cv.putText(mask,'hit escape once you are satisfied',(0,70), font, 1,(255,255,255),2,cv.LINE_AA)
        cv.imshow('mask',mask)
        if cv.waitKey(20) & 0xFF == 27:#waitkey usage from https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html
            break
    cv.destroyWindow('sliders')
    cv.destroyWindow('mask')
    
    return [hMin,sMin,vMin],[hMax,vMax,sMax]

#returns standard deviation of a list
def stDev(data):
    mean=average(data)
    tempSum=0
    for val in data:
        tempSum+=(val-mean)**2
    return math.sqrt(tempSum/(len(data)-1))

#returns average of a list
def average(lst):
    return sum(lst)/len(lst)

def calibrateLength(minVal, maxVal):
    saberSizes=[]
    error=10
    font=cv.FONT_HERSHEY_SIMPLEX
    sizeCalibrated=False
    white=(255,255,255)
    while(not sizeCalibrated):
        if sizeCalibrated:
            break
        _, frame = cap.read()
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        hsv=cv.medianBlur(hsv,11)
        mask = cv.inRange(hsv, minVal, maxVal)
        res = cv.bitwise_and(frame,frame, mask= mask)#from https://docs.opencv.org/master/d0/d86/tutorial_py_image_arithmetics.html
        frame=cv.flip(frame,1)
        mask=cv.flip(mask,1)
        res=cv.flip(res,1)
        #next 2 lines based on guide on morphological transform: https://docs.opencv.org/master/d9/d61/tutorial_py_morphological_ops.html
        kernel = np.ones((13,13),np.uint8)
        mask=cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        if len(contours)>0:# a contour is detected
            #contour detection next 4 lines from, https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html
            rect = cv.minAreaRect(contours[0])
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.drawContours(res,[box],0,(0,0,255),2)
            x0,y0=box[0]
            x1,y1=box[1]
            x2,y2=box[2]
            saberLength=max(distance(x0,y0,x1,y1),distance(x1,y1,x2,y2))
            saberSizes.append(saberLength)
            cv.putText(res,f'Try to keep this number as stable as possible: ->{saberLength}',(0,30), font, 1,white,2,cv.LINE_AA)
            cv.putText(res,'Window will automatically close when calibrated',(0,70), font, 1,white,2,cv.LINE_AA)
        cv.imshow('res', res)
        if cv.waitKey(20) & 0xFF == 27:#waitkey usage from https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html
            break
        
        elif(len(saberSizes)>25):
            dataError=stDev(saberSizes)
            if(dataError<error):
                sizeCalibrated=True 
                cv.destroyWindow('res')
                return average(saberSizes)
            else:
                saberSizes=[]
    cv.destroyWindow('res')
    return average(saberSizes)

def getMidpoint(x0,y0,x1,y1):
    cx=(x0+x1)/2
    cy=(y0+y1)/2
    return(cx,cy)

def calibrateLighting():
    H=[]
    S=[]
    V=[]
    minVal,maxVal=calibrateSaberClick()
    calMin,calMax=calibrateSaberSlide(minVal,maxVal)
    lowerHSV=np.array(calMin)
    upperHSV=np.array(calMax)
    return lowerHSV, upperHSV

def noteHittable(closestFrames,gridCenters):
   
    for frame in closestFrames:
        if frame==None or frame[2]==-1:
            pass
        else:
            row,col,direction=frame
            pos=str(row)+str(col)
            x,y=gridCenters[pos]
            return x,y,direction, True
    return 0,0,0,False

def bombHittable(closestFrames,gridCenters):
    for frame in closestFrames:
        if frame==None or frame[2]!=-1:
            pass
        else:
            row,col,val=frame
            pos=str(row)+str(col)
            x,y=gridCenters[pos]
            return x,y,True
    return 0,0,False

#convert the saber into a list of 10 tuples for object hit detection
def makeLine(x0,y0,x1,y1,resolution):
    m=(y1-y0)/(x1-x0)
    dx=(x1-x0)/resolution
    result=[(x0,y0)]
    for i in range(resolution):
        newX=x0+i*dx
        newY=y0+i*m
        if math.isnan(newX) or math.isinf(newX):#learned about nan/inf from https://stackoverflow.com/questions/944700/how-can-i-check-for-nan-values
            newX=0
        if math.isnan(newY) or math.isinf(newX):
            newY=0
        result.append((newX,newY))
    return result
        

#general function, acts as 'main' for the cv loop
def getCenter(box):
    x0,y0=box[0]
    x1,y1=box[1]
    x2,y2=box[2]
    x3,y3=box[3]
    minX=min(x0,x1,x2,x3)
    maxX=max(x0,x1,x2,x3)
    minY=min(y0,y1,y2,y3)
    maxY=max(y0,y1,y2,y3)
    cx,cy=getMidpoint(minX,minY,maxX,maxY)
    return cx,cy

lower_blue = np.array([62,91,82])
upper_blue = np.array([123,255,255])   
#pass in the entire map, and create a map decoder in the saberTracker
def generalTracking(levelMap,noteVisibility,lowerHSV=lower_blue, upperHSV=upper_blue, trueSaberLength=0):
    calibrated=False
    noteNear=False
    swingThreshold=4
    score=0
    combo=0
    centers=[]
    XY=[]
    YZ=[]
    XZ=[]
    camWidth  = cap.get(3)
    camHeight = int(cap.get(4))
    cxPrev=0
    cyPrev=0
    mapLength=len(levelMap)
    gridWidth=camHeight//3
    gridHeight=gridWidth
    i=0
    gridCenters=dict()
    gridCenters['00']=(camWidth/2-gridWidth,camHeight/2-gridHeight)
    gridCenters['01']=(camWidth/2,camHeight/2-gridHeight)
    gridCenters['02']=(camWidth/2+gridWidth,camHeight/2-gridHeight)
    gridCenters['10']=(camWidth/2-gridWidth,camHeight/2)
    gridCenters['11']=(camWidth/2,camHeight/2)
    gridCenters['12']=(camWidth/2+gridWidth,camHeight/2)
    gridCenters['20']=(camWidth/2-gridWidth,camHeight/2+gridHeight)
    gridCenters['21']=(camWidth/2,camHeight/2+gridHeight)
    gridCenters['22']=(camWidth/2+gridWidth,camHeight/2+gridHeight)
    noteSize=gridWidth
    font = cv.FONT_HERSHEY_SIMPLEX
    blue=(255,0,0)
    white=(255,255,255)
    gray=(101,101,101)

    #stuff for note slicing particle effects
    noteParticles=[]
    while(True):
        startTime=time.time()
        _, frame = cap.read()
        # Convert BGR to HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

        hsv=cv.medianBlur(hsv,11)

        mask = cv.inRange(hsv, lowerHSV, upperHSV)
        
        kernel = np.ones((13,13),np.uint8)
        mask=cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        notes=np.zeros((720,1280,3), np.uint8)
        #flip each image vertically
        res = cv.bitwise_and(frame,frame, mask= mask)#overlay boolean mask onto frame, from https://docs.opencv.org/master/d0/d86/tutorial_py_image_arithmetics.html
        frame=cv.flip(frame, 1)
        mask = cv.flip(mask, 1)
        res=cv.flip(res,1)

        #identify saber w/ contour detection, https://docs.opencv.org/3.4/d4/d73/tutorial_py_contours_begin.html
        contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        
        #draw the notes onto the result
        visibleNotes=levelMap[0:noteVisibility]
        notesSeen=len(visibleNotes)
        smallGap=100
        bigGap=300
        topOffset=int(camHeight/2)
        lineColor=(255,255,0)
        cv.line(res,(int(camWidth/2-smallGap),topOffset),(int(camWidth/2-bigGap),camHeight),lineColor,5)
        cv.line(res,(int(camWidth/2+smallGap),topOffset),(int(camWidth/2+bigGap),camHeight),lineColor,5)
        bombRad=0
        for i in range(notesSeen):
            value=visibleNotes[i]
            if value!=None:
                row,col,direction=value
                sizeRatio=((notesSeen-i)/notesSeen)
                size=noteSize*sizeRatio
                pos=str(row)+str(col)
                x,y=gridCenters[pos]
                dx=x-camWidth/2
                cx=int(camWidth/2+dx*sizeRatio)
                dy=y-camHeight/2
                cy=int(camHeight/2+dy*sizeRatio)
                if direction==-1:
                    cv.circle(res,(cx,cy), int(size/2), gray,-1)
                    if bombRad<size/2:
                        bombRad=size/2
                else:
                    
                    topLeft=(int(cx-size/2), int(cy-size/2))
                    botRight=(int(cx+size/2),int(cy+size/2))
                    cv.rectangle(notes,topLeft,botRight,blue,-1)
                    cv.putText(notes, '>',(cx,cy),font,2*size/noteSize,white,2,cv.LINE_AA)
                    #text rotation code(next 2 lines) from https://theailearner.com/2020/11/02/how-to-write-rotated-text-using-opencv-python/
                    M = cv.getRotationMatrix2D((cx,cy), direction, 1)
                    notes = cv.warpAffine(notes, M, (notes.shape[1], notes.shape[0]))

                    #update result with notes, and reset notes to blank
                    res=cv.add(notes,res)
                    notes=np.zeros((720,1280,3), np.uint8)

        #checks if the closest frame contains a note
        closestFrames=levelMap[:swingThreshold]
        noteX,noteY,direction,noteNear=noteHittable(closestFrames,gridCenters)
        bombX,bombY,bombNear=bombHittable(closestFrames,gridCenters)
        levelMap=levelMap[1:]
        cv.putText(res,f'score: {score}',(10,300), font, 1,white,2,cv.LINE_AA)
        cv.putText(res,f'combo:{combo}',(10,400), font, 1,white,2,cv.LINE_AA)

        #code to draw progress bar
        pos1=(int(camWidth-camWidth//5),int(camHeight-30))
        pos2=(int(camWidth-camWidth//15),int(camHeight-10))
        barWidth=(camWidth-camWidth//15)-(camWidth-camWidth//5)
        cv.rectangle(res,pos1,pos2,white,2)
        progRatio=1-(len(levelMap)/mapLength)
        cv.putText(res,f'Map Progress:',(int(camWidth-camWidth//5),650), font, 1,white,2,cv.LINE_AA)
        progress1=(int((camWidth-camWidth//5)),int(camHeight-30))
        progress2=(int((camWidth-camWidth//15)-barWidth+(progRatio*barWidth)),int(camHeight-10))
        cv.rectangle(res,progress1,progress2,gray,-1)

        #code to draw the particle effects:
        i=0
        maxFrames=15
        decel=20
        while i <len(noteParticles):
            x,y,frames,direction,v=noteParticles[i]
            
            if frames>maxFrames:
                noteParticles.pop(i)
            else:
                
                frames+=1
                #draws circle effects
                if v>500:v=500
                x+=v/decel*math.cos(math.radians(direction))
                y+=-v/decel*math.sin(math.radians(direction))
                #decelerate a bit:
                if v>decel:
                    v-=decel
                for j in range(3):
                    #more spread as particles get 'older'
                    randX=random.randint(-5*frames,5*frames)
                    randY=random.randint(-5*frames,5*frames)
                    size=15
                    cv.circle(res,(int(x+randX),int(y+randY)), size, blue,-1)
                noteParticles[i]=x,y,frames,direction,v
                i+=1

        if len(contours)>0:#we see a saber in our frame
            #next 4 lines from here:# https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html
            rect = cv.minAreaRect(contours[0])
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.drawContours(res,[box],0,(0,255,0),1)

            cx,cy=getCenter(box)
            #determine length of saber
            x0,y0=box[0]
            x1,y1=box[1]
            x2,y2=box[2]
            saberLength1=distance(x0,y0,x1,y1)
            saberLength2=distance(x1,y1,x2,y2)
            
            if(saberLength1>saberLength2):
                saberLength=saberLength1
                saberWidth=saberLength2
                saberVector2D=np.array([x1-x0,y0-y1])
                saberLine=(makeLine(x0,y0,x1,y1,10))
            else:
                saberLength=saberLength2
                saberWidth=saberLength1
                saberVector2D=np.array([x2-x1,y1-y2])
                saberLine=makeLine(x1,y1,x2,y2,10)

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
            #cv.putText(mask,f'{xyAngle}',(10,300), font, 1,white,2,cv.LINE_AA)
            
            #check if we have hit a note
            if len(levelMap)!=0 and ((levelMap[0]==None) or (levelMap[0][2]==-1)):
                nextFrameHasNote=False
            elif len(levelMap)!=0:
                nextFrameHasNote=True
            else:
                break
                
            inSwing=isSwinging(centers) and isSwing3D(XY,YZ,XZ,centers,dt)

            if inSwing and bombNear:#we swung while a bomb is near
                bombHit=False
                if distance(cx,cy,bombX,bombY)<bombRad:
                # for x,y in saberLine:
                #     if type(x)==float and type(y)==float:
                #         if distance(x,y,bombX,bombY)<bombRad:
                            bombHit=True
                #convert saber to a 'line', which is basically just 10 (x,y) tuples
                #for the x and y in the line, if it's distance is less than bombRad, then bombHit is true 
                if(bombHit):
                    pygame.mixer.Channel(3).play(missed)
                    score-=500
                    if score<0:
                        score=0
                    #if so, subtract 500 from the score if possible
                    for i in range (len(levelMap)):
                        frame=levelMap[i]
                        if frame!=None and frame[2]==-1:
                            levelMap[i]=None
                            break
                    #remove bomb from map
                

            if inSwing and noteNear:
                #we swung while a note is near
                if abs(direction-xyAngle)<15 or abs(direction-xyAngle-180)<15:
                    
                    score+=100
                    combo+=1
                    pygame.mixer.Channel(1).play(hit)
                    
                    firstPos=centers[-1]
                    secPos=centers [-2]
                    dv=distance(firstPos[0],secPos[0],firstPos[1],secPos[1])
                    noteParticles.append((noteX,noteY,0,direction,dv))
                    #remove the note from the map so you can't hit it again
                    for i in range (len(levelMap)):
                        frame=levelMap[i]
                        if frame!=None and frame[2]!=-1:
                            levelMap[i]=None
                            i=len(levelMap)-1
                            break
                else:#we sliced in the wrong direction
                    pygame.mixer.Channel(1).play(missed)
                    combo=0
            elif not inSwing and nextFrameHasNote:#we let a note fly by
                pygame.mixer.Channel(2).play(missed)
                combo=0
            
            #debugging tools
            #print(cx,cy)
            #print(f'pitch: {yzAngle}, yaw: {xzAngle}, plane angle: {xyAngle}')
            #cv.putText(res,f'dx: {cx-cxPrev}, dy: {cy-cyPrev}',(10,300), font, 1,(255,255,255),2,cv.LINE_AA)
            cv.putText(mask,f'pitch: {yzAngle}, yaw: {xzAngle}, roll:{xyAngle}',(10,300), font, 1,(255,255,255),2,cv.LINE_AA)
            #cv.putText(res,f'{saberVector}',(10,300), font, 1,(255,255,255),2,cv.LINE_AA)
            cxPrev=cx
            cyPrev=cy
            startTime=time.time()
        
        
        cv.imshow('camera',frame) 
        #cv.imshow('mask',mask)
        cv.imshow('gameScreen', res)

        if cv.waitKey(5) & 0xFF == 27:#waitkey usage from https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html
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

def isSwing3D(XY,YZ,XZ,centers,dt):
    #Change in center position indicates saber is moving in isSwinging
    #derivative of angles is then checked to make sure we are actually swinging and not translating
    #although it could all be done through looking at centers, analyzing the saber's angles will ensure a swing is proper and not just a translation

    #shorten to the most recent values for the swing
    minTime=3
    shortenedXY=XY[-minTime:]
    shortenedYZ=YZ[-minTime:]
    shortenedXZ=XZ[-minTime:]
    
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

    minMovement=8
    minSpread=3
    
    #using each angle's derivative wrt time, and the standard deviations of the past few 'minTime' values, we find the swing direction
    if ((xzSpread>minMovement>minSpread and dXZdt>minMovement) or
    (yzSpread>minMovement>minSpread and dYZdt>minMovement)):
        return True
    else:
        return False

    pass
def debugScreen(lowerHSV=lower_blue, upperHSV=upper_blue, trueSaberLength=0):
    while(True):
        font = cv.FONT_HERSHEY_SIMPLEX
        _, frame = cap.read()
        
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        hsv=cv.medianBlur(hsv,11)
        mask = cv.inRange(hsv, lowerHSV, upperHSV)
        res = cv.bitwise_and(frame,frame, mask= mask)
        frame=cv.flip(frame,1)
        mask=cv.flip(mask,1)
        res=cv.flip(res,1)
        kernel = np.ones((13,13),np.uint8)
        mask=cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        
        
        contours, hierarchy = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        if len(contours)>0:
            #next 4 lines from here:# https://docs.opencv.org/3.4/dd/d49/tutorial_py_contour_features.html
            rect = cv.minAreaRect(contours[0])
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.drawContours(res,[box],0,(0,255,0),1)
        cv.putText(res,'hit escape once you are finished testing',(10,50), font, 1,(255,255,255),2,cv.LINE_AA)
        cv.imshow('result',res)
        if cv.waitKey(5) & 0xFF == 27:#waitkey usage from https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html
                break
    cv.destroyAllWindows()

