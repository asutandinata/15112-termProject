import pygame
from cmu_112_graphics import *
pygame.init()
backgroundMusic=pygame.mixer.Sound('testMusic.mp3')
soundEffect=pygame.mixer.Sound('Eating.wav')

def appStarted(app):
    app.backgroundMusicPaused=True
    app.ballX=app.width/2
    app.ballY=app.height/3
def timerFired(app):
    print(pygame.mixer.Channel(0).get_volume())
    app.ballX+=5
    if(app.ballX==app.width):
        app.ballX=0
def keyPressed(app, event):#https://www.pygame.org/docs/ref/mixer.html#pygame.mixer.Channel
    if event.key=='p':
        if(app.backgroundMusicPaused):
            pygame.mixer.Channel(0).unpause()
            pygame.mixer.Channel(0).play(backgroundMusic)
            app.backgroundMusicPaused=False
        else:
            pygame.mixer.Channel(0).pause()
            app.backgroundMusicPaused=True
    elif event.key=='o':
        pygame.mixer.Channel(1).play(soundEffect)
    elif(event.key=='q'):
        pygame.mixer.stop()
        app.backgroundMusicPaused=False

def redrawAll(app, canvas):
    canvas.create_text(app.width/2, app.height/2,text=f'Music Playing: {app.backgroundMusicPaused}')
    canvas.create_oval(app.ballX-10, app.ballY-10, app.ballX+10, app.ballY+10, fill='red')
runApp(width=400, height=400)


