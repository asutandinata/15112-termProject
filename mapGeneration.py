import random
import math

#tuple denotes each "frame" and is stored in a larger array(levelMap)
#each frame contains a tuple containing the row, column and direction of a beat
#ie:(None) represents an empty Frame
#(0,1,0) represents a beat in the top middle where you have to swing up
#(0,1,90) represents a beat in the top middle where you have to swing right

#maps will be generated randomly

#rules for creating a frame:
    #each frame cannot have a note going outwards in, it must be inwards out
        #ie, corner notes can only have a note going out in their respective corner direction
        #a note in the center can be going in any direction
        #notes in a "cardinal direction" have 3 options

    #a note position cannot be the exact same as a previous note, that'd be too boring

    #if we decide to spawn a bomb, the bomb cannot be right before a note frame
        #there must be some amount of tolerance

#bombs cannot be touched by a saber
#bombs will be denoted by (row,col,-1)
#ie (1,1,-1) represents a bomb in the exact center

#values needed to generate a map:
    #length in frames
    #maximum beats in map
    #the amount of empty frames between notes
    #bombs enabled on the map
    #chance of spawning a bomb, 0 by default
        #represents a '1 in x' chance of spawning a bomb in an empty frame

#things added to increase complexity after TP2, regarding map generation:

#make the gap in between note frames more random: 
# make the next note direction based on the previous note direction

#introduce streams back and forth of ups and downs

#make bomb generation look cooler(not just random)

#passive swing patterns:
#big swing, if a piece is straight up or down in the middle row, make another note right after in the same direction
#dumb stream, the note right after is a swing in the oppisite direction

#fun swing patterns for added complexity(actively called on)
#"fun" pattern triggered by a random event
#3 main options:
#spiral clockwise and counterclockwise, create a bomb in the center frame
#"jail bars"(up in top left followed by down in mid bottom, followd by up in top right)
#"sideways jail bars"
#streams, rapid up and down swinging(no left or right)
#


#general map making function
#on pass 1 introduce notes
#on pass 2 introduce pssive note features
#on pass 3 introduce fun patterns and bombs if enabled
def generateMap(frames,OGframeGap, bombsEnabled, bombChance=0, funPatterns=False):
    mapFrames=30*[None]
    previousFrame=(-1,-1,-1)
    gapTolerance=5
    framesSinceLastNote=0
    for i in range(frames):
        frameGap=random.randint(OGframeGap-gapTolerance, OGframeGap+gapTolerance)
        if(framesSinceLastNote>=frameGap):
            newFrame=generateFrame(previousFrame)
            previousFrame=newFrame
            mapFrames.append(newFrame)
            framesSinceLastNote=-1
        else:
            mapFrames.append(None)
        framesSinceLastNote+=1
    passiveFeatures(mapFrames)
    #if bombs are enabled, sprinkle them into random spots on emptyFrames
    if funPatterns:
        generateFunPatterns(mapFrames)
    if bombsEnabled:
        generateBombs(mapFrames, bombChance)
    return mapFrames

def generateFrame(previousFrame):
    row=random.randint(0,2)
    col=random.randint(0,2)
    if row==previousFrame[0] or col==previousFrame[1]:
        return generateFrame(previousFrame)
    #we know we have a valid row and column position
    else:
        directions=[0,45,90,135,180,225,270,315]
        
        if col==0:
            #swings can only be outwards:[135,180,225]
            directions.remove(0)
            directions.remove(45)
            directions.remove(315)
            if row==1:
                pass
            elif row==0:
                directions.remove(180)
                directions.remove(225)
                directions.remove(90)
                directions.remove(270)
            elif row==2:
                directions.remove(135)
                directions.remove(180)
                directions.remove(90)
                directions.remove(270)
        elif col==2:
            #swings cannot be leftwardsmust be to the right[0,45,315]
            directions.remove(135)
            directions.remove(180)
            directions.remove(225)
            
            
            if row==1:
                pass
            elif row==0:
                directions.remove(0)
                directions.remove(315)
                directions.remove(90)
                directions.remove(270)
                
            elif row==2:
                directions.remove(0)
                directions.remove(45)
                directions.remove(90)
                directions.remove(270)
                
        elif row==0:
            #swings can only be upwards:[45,90,135]
            directions.remove(0)
            directions.remove(180)
            directions.remove(225)
            directions.remove(270)
            directions.remove(315)
        elif row==2:
            #swings can only be downwards:[225,270,315]
            directions.remove(0)
            directions.remove(45)
            directions.remove(90)
            directions.remove(135)
            directions.remove(180)

        i=random.randint(0,len(directions)-1)
        direction=directions[i]

        return (row,col, direction)

def generateBombs(mapFrames, bombChance):
    for i in range(len(mapFrames)-2):
        frame=mapFrames[i]
        nextFrame=mapFrames[i+1]
        nextNextFrame=mapFrames[i+2]
        if frame==(None) and nextFrame==(None) and nextNextFrame==(None):
            if(1==random.randint(0,bombChance)):
                    row=random.randint(0,2)
                    col=random.randint(0,2)
                    mapFrames[i]=(row,col,-1)

#big swing, if a piece is straight up or down in the middle row, make another note right after in the same direction
#dumb stream, the note right after is a swing in the oppisite direction, only occurs in the middle note
#passive note features will be very likely to occur as the make the game much more interesting
def passiveFeatures(mapFrames):
    dumbChance=1
    bigSwingChance=1
    for i in range(len(mapFrames)):
        frame=mapFrames[i]
        if frame!=None:
            if frame[1]==1 and frame[0]==1 and random.randint(0,dumbChance)==1:#there is a center note, make a dumb stream
                mapFrames[i+1]==(0,0,frame[2]+180)
            elif frame[0]==1 :
                #generate a note in the same direction either above or below it, depending on direction
                if frame[2]==90 and (frame[0]==1 or frame[0]==2):#swinging up
                    mapFrames[i+1]=(frame[0]-1,frame[1],frame[2])
                elif frame[2]==270 and (frame[0]==1 or frame[0]==0):
                    mapFrames[i+1]=(frame[0]+1,frame[1],frame[2])
def generateFunPatterns(mapFrames):
    #each 'fun' pattern is 8 notes long
    #we search for any gaps of 8 empty notes
    i=0
    while i< len(mapFrames)-12:
        gapFound=False
        for j in range(12):
            if mapFrames[i+j]!=None:
                gapFound=True
        if gapFound:
            n=random.randint(0,3)
            #create one of the 3 random patterns starting at i
            if n==0:#generate a clockwise spiral
                startRow,startCol=random.choice([(0,1),(1,0),(1,2),(2,1)])
                makeSpiral(mapFrames,i,startRow,startCol,'cw')
            elif n==1:#generate a counterclockwiseSpiral
                startRow,startCol=random.choice([(0,1),(1,0),(1,2),(2,1)])
                makeSpiral(mapFrames,i,startRow,startCol,'ccw')
            elif n==2:#vertical bars
                startRow=random.choice([0,2])
                startCol=random.choice([0,2])
                makeBarsVertical(mapFrames,i,startRow,startCol)
            elif n==3:#horizontal bars
                startRow=random.choice([0,2])
                startCol=random.choice([0,2])
                makeBarsHorizontal(mapFrames,i,startRow,startCol)
            i+=12
        i+=1
    pass
                
def makeSpiral(mapFrame,i,row,col,direction):
    directionsCCW={'12':90, '21':0,'10':270,'01':180}
    directionsCW={'12':270, '21':180,'10':90,'01':0}
    for j in range(12):
        if j>9:
            mapFrame[i+j]=None
        elif direction=='ccw':
            if (i+j)%2==1:
                pointing=str(row)+str(col)
                mapFrame[i+j]=(row,col,directionsCCW[pointing])
                if row==1 and col==2:
                    col=1
                    row=0
                elif row==0 and col==1:
                    row=1
                    col=0
                elif row==1 and col==0:
                    col=1
                    row=2
                elif row==2 and col==1:
                    row=1
                    col=2
            else:
                mapFrame[i+j]=(1,1,-1)
        else:#spinning clockwise
            if j%2==1:
                pointing=str(row)+str(col)
                mapFrame[i+j]=(row,col,directionsCW[pointing])
                if row==1 and col==2:
                    col=1
                    row=2
                elif row==0 and col==1:
                    row=1
                    col=2
                elif row==1 and col==0:
                    col=0
                    row=1
                elif row==2 and col==1:
                    row=1
                    col=0
            else:
                mapFrame[i+j]=(1,1,-1)
        
def makeBarsVertical(mapFrame,i,row,col):
    if row==0:
        swingDir=90
    else:
        swingDir=270
    if col==0:
        dx=1
    else:
        dx=-1
    for j in range(12):
        if j%2==1 or j>=4:
            mapFrame[i+j]=None
        else:
            mapFrame[i+j]=(row,col,swingDir)
            if row==0:
                row=2
            else:
                row=0
            if swingDir==270:
                swindDir=90
            else:
                swingDir=270
            col+=dx

def makeBarsHorizontal(mapFrame,i, row,col):
    if col==0:
        swingDir=180
    else:
        swingDir=0
    if row==0:
        dy=1
    else:
        dy=-1
    print(row,col,swingDir)
    for j in range(12):
        if j%2==1 or j>=4:
            mapFrame[i+j]=None
        else:
            if col==0:
                col=2
            else:
                col=0
            if swingDir==180:
                swindDir=0
            else:
                swingDir=180
            print(row,col,swingDir)
            mapFrame[i+j]=(row,col,swingDir)
            col+=dy
            
                
a=generateMap(750, 20,False,0,True)
