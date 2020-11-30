from cmu_112_graphics import *
import 'openCV test'
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

    
def createButtons(app):
    playButton=button(app.width/2, app.height/4, app.width/2, 0.2*app.height, 'play a song')
    calibrateButton=button(app.width/2, app.height/2, app.width/2, 0.2*app.height, 'Calibrate your sabers')
    helpButton=button(app.width/2, 3*app.height/4, app.width/2, 0.2*app.height, 'help')
    app.homeButtons=[playButton, calibrateButton, helpButton]

    back=button(app.width/7, app.height/5, 100, 50, 'Back')
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
            elif button.text=='play a song':
                app.displayButtons=app.levelButtons
            elif button.text=='Calibrate your sabers':
                app.displayButtons=app.calibrateButtons
            elif button.text=='help':
                app.displayButtons=app.helpButtons
            elif button.text=='Calibrate Lighting':
                #calibrate the lighting
                pass
            elif button.text=='Calibrate Length':
                #calibrate the length
                pass
            elif button.text=='Debug':
                #debug the stuff
                pass

def redrawAll(app, canvas):
    for button in app.displayButtons:
        cx=button.cx
        cy=button.cy
        width=button.width
        height=button.height
        text=button.text
        canvas.create_rectangle(cx-width/2, cy-height/2, cx+width/2, cy+height/2,fill='yellow')
        canvas.create_text(cx, cy, text=text, fill='black', font='arial 12 bold')

runApp(width=800, height=400)
