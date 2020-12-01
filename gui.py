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
    app.frameSpeed=0
    
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

def redrawAll(app, canvas):
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
runApp(width=800, height=400)
