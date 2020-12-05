import random

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



def generateMap(frames,frameGap, bombsEnabled, bombChance=0):
    mapFrames=20*[None]
    previousFrame=(-1,-1,-1)
    framesSinceLastNote=0
    for i in range(frames):
        if(framesSinceLastNote==frameGap):
            beatsSinceFrame=0
            newFrame=generateFrame(previousFrame)
            previousFrame=newFrame
            mapFrames.append(newFrame)
            framesSinceLastNote=-1
        else:
            mapFrames.append(None)
        framesSinceLastNote+=1
    #if bombs are enabloed, sprinkle them into random spots on emptyFrames
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
            directions.remove(90)
            directions.remove(270)
            directions.remove(315)
            if row==1:
                pass
            elif row==0:
                directions.remove(180)
                directions.remove(225)
            elif row==2:
                directions.remove(135)
                directions.remove(180)
        elif col==2:
            #swings must be to the right[0,45,315]
            directions.remove(90)
            directions.remove(135)
            directions.remove(180)
            directions.remove(225)
            directions.remove(270)
            
            if row==1:
                pass
            elif row==0:
                directions.remove(0)
                directions.remove(315)
            elif row==2:
                directions.remove(0)
                directions.remove(45)
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



   