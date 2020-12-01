from cmu_112_graphics import *
import saberTracker
import mapGeneration
import numpy as np
import time

class button(object):
    def __init__(self, x,y, width,height,text):
        self.cx=x
        self.cy=y
        self.width=width
        self.height=height
        self.text=text
    def __hash__(self):
        return hash(self.x)

def appStarted(app):
    app.calibrated=False
    app.showBackButton=False
    app.homeButtons=[]
    app.calibrateButtons=[]
    app.difficultyButtons=[]
    app.helpButtons=[]
    createButtons(app)
    app.displayButtons=app.homeButtons
    app.heading='Welcome to cheapsaber!'

    #errors:
    app.showError=False
    app.errorText=''

    #cv variables 
    app.lowerHSV=None
    app.upperHSV=None
    app.calibrated=False
    app.trueSaberLength=0

    #map variables:
    app.levelMap=[]
    app.noteSpeed=0
    app.noteVisibility=10#how many notes into the future you can see
    app.score=0
    app.combo=0
    app.inGame=False
    
    #note drawing variables:
    determineNoteGrid(app)

def determineNoteGrid(app):
    gridWidth=app.height//3
    gridHeight=gridWidth
    app.gridCenters=dict()
    app.gridCenters['00']=(app.width/2-gridWidth,app.height/2-gridHeight)
    app.gridCenters['01']=(app.width/2,app.height/2-gridHeight)
    app.gridCenters['02']=(app.width/2+gridWidth,app.height/2-gridHeight)
    app.gridCenters['10']=(app.width/2-gridWidth,app.height/2)
    app.gridCenters['11']=(app.width/2,app.height/2)
    app.gridCenters['12']=(app.width/2+gridWidth,app.height/2)
    app.gridCenters['20']=(app.width/2-gridWidth,app.height/2+gridHeight)
    app.gridCenters['21']=(app.width/2,app.height/2+gridHeight)
    app.gridCenters['22']=(app.width/2+gridWidth,app.height/2+gridHeight)
    app.noteSize=gridWidth


def createButtons(app):
    playButton=button(app.width/2, app.height/4, app.width/2, 0.2*app.height, 'play a song')
    calibrateButton=button(app.width/2, app.height/2, app.width/2, 0.2*app.height, 'Calibrate your sabers')
    helpButton=button(app.width/2, 3*app.height/4, app.width/2, 0.2*app.height, 'help')
    app.homeButtons=[playButton, calibrateButton, helpButton]

    back=button(55, 30, 100, 50, 'Back')
    easy=button(app.width/2, app.height/5, app.width-150, 0.2*app.height, 'Easy')
    medium=button(app.width/2, 2*app.height/5, app.width-150, 0.2*app.height, 'Medium')
    hard=button(app.width/2, 3*app.height/5, app.width-150, 0.2*app.height, 'Hard')
    expert=button(app.width/2, 4*app.height/5, app.width-150, 0.2*app.height, 'Expert')
    app.levelButtons=[easy, medium, hard, expert,back]

    calLighting=button(app.width/2, app.height/4, app.width-150, 0.2*app.height, 'Calibrate Lighting')
    calLength=button(app.width/2, app.height/2, app.width-150, 0.2*app.height, 'Calibrate Length')
    debug=button(app.width/2, 3*app.height/4, app.width-150, 0.2*app.height, 'Debug')
    app.calibrateButtons=[calLighting, calLength, back,debug]

    app.helpButtons=[back]
def mousePressed(app, event):
    for button in app.displayButtons:
        if (event.x>(button.cx-button.width/2) and event.x<(button.cx+button.width/2)
            and event.y>(button.cy-button.height/2) and event.y<(button.cy+button.height/2)):
            if button.text=='Back':
                app.displayButtons=app.homeButtons
                app.heading='Welcome to cheapsaber!'
                app.showError=False
                app.error=''
            elif button.text=='play a song':
                app.displayButtons=app.levelButtons
                app.heading='Select a level'
                if(not app.calibrated):
                    app.showError=True
                    app.errorText='You have not calibrated!\n Please go back and calibrate your saber'
            elif button.text=='Calibrate your sabers':
                app.displayButtons=app.calibrateButtons
                app.heading='calibrate your saber'
            elif button.text=='help':
                app.displayButtons=app.helpButtons
                app.heading='Help Screen'
            elif button.text=='Calibrate Lighting':
                app.lowerHSV,app.upperHSV=saberTracker.calibrateLighting()
            elif button.text=='Calibrate Length':
                app.trueSaberLength=saberTracker.calibrateLength(app.lowerHSV, app.upperHSV)
                app.calibrated=True
            elif button.text=='Debug':
                if app.calibrated:
                    saberTracker.generalTracking(app.lowerHSV, app.upperHSV, app.trueSaberLength)
                else:
                    pass
            elif button.text=='Easy':
                app.levelMap=mapGeneration.generateMap(500, 8,False)
                app.noteSpeed=5
                app.noteVisibility=10
                app.inGame=True
                
            elif button.text=='Medium':
                app.levelMap=mapGeneration.generateMap(250, 8,True,10)
                app.noteSpeed=7
                app.noteVisibility=10
                app.inGame=True
                
            elif button.text=='Hard':
                app.levelMap=mapGeneration.generateMap(1000, 6,True,10)
                app.noteSpeed=8
                app.noteVisibility=10
                app.inGame=True
                
            elif button.text=='Expert':
                app.levelMap=mapGeneration.generateMap(1250, 4,True,9)
                app.noteSpeed=10
                app.noteVisibility=20
                app.inGame=True
                

def timerFired(app):
    app.timerDelay=10*(11-app.noteSpeed)+50
    if app.inGame:
        app.levelMap=app.levelMap[1:]
        if len(app.levelMap)==0:
            app.inGame=False


def drawNotes(app, canvas):
    visibleNotes=app.levelMap[0:app.noteVisibility]
    notesSeen=len(visibleNotes)
    for i in range (notesSeen):
        value=visibleNotes[i]
        if value!=None:
            row,col,direction=value
            sizeRatio=((notesSeen-i)/notesSeen)
            size=app.noteSize*sizeRatio
            pos=str(row)+str(col)
            x,y=app.gridCenters[pos]
            dx=x-app.width/2
            cx=app.width/2+dx*sizeRatio
            dy=y-app.height/2
            cy=app.height/2+dy*sizeRatio
            if direction==-1:
                canvas.create_oval(cx-size/2, cy-size/2,cx+size/2,cy+size/2, fill='gray')
            else:
                canvas.create_rectangle(cx-size/2, cy-size/2,cx+size/2,cy+size/2, fill='blue',outline='black')
                canvas.create_text(cx,cy,text='^', font=f'arial {int(size/2)}',angle=int(direction)+180, fill='white')
                
        
def redrawAll(app, canvas):
    if not app.inGame:
        canvas.create_text(app.width/2, 15,text=app.heading, fill='black', font='arial 18 bold')
        for button in app.displayButtons:
            cx=button.cx
            cy=button.cy
            width=button.width
            height=button.height
            text=button.text
            canvas.create_rectangle(cx-width/2, cy-height/2, cx+width/2, cy+height/2,fill='yellow')
            canvas.create_text(cx, cy, text=text, fill='black', font='arial 12 bold')
        if app.showError:
            margin=20
            canvas.create_rectangle(margin, margin, app.width-margin, app.height-margin,fill='red')
            canvas.create_text(app.width/2, app.height/2, text=app.errorText, fill='black', font='arial 20 bold')
    else:
        #create backdrop and stuff
        canvas.create_rectangle(0,0,app.width,app.height,fill='black')
        smallGap=100
        bigGap=500
        topOffset=app.height/2
        canvas.create_line(app.width/2-smallGap, topOffset, app.width/2-bigGap, app.height, fill='cyan',width=5)
        canvas.create_line(app.width/2+smallGap, topOffset, app.width/2+bigGap, app.height, fill='cyan',width=5)
        drawNotes(app, canvas)
runApp(width=1280, height=960)
