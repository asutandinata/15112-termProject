import numpy as np
import cv2 as cv


H=[]
S=[]
V=[]

def getValueOnClick(event,x,y,flags,hsv):
    if event == cv.EVENT_LBUTTONDOWN:
        print(hsv.shape)
        print(y,x)
        
        HSVval=(hsv[y][x])
        H.append(HSVval[0])
        S.append(HSVval[1])
        V.append(HSVval[2])

cap = cv.VideoCapture(0)

def begin():
    justClicked=False
    while len(H)<4:
        _, frame = cap.read()
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        hsv=cv.medianBlur(hsv,11)
        frame=cv.flip(frame, 1)
        hsv=cv.flip(frame,1)
        cv.setMouseCallback('frame',getValueOnClick,frame)
        cv.imshow('frame',frame)
        if cv.waitKey(20) & 0xFF == 27:
            break
    minVal=[min(H), min(S), min(V)]
    maxVal=[max(H),max(S),max(V)]
    return minVal,maxVal


print(begin())
cv.destroyAllWindows()
