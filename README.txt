Welcome: this game is copy of the popular vr game "beatsaber", but only using computer hardware you already have and items you can find around the house. 
To play this game, your goal is to swing the saber at a note in a certain direction, as it comes toward you. Notes will be either standalone, or in patterns.
Modules to install:
opencv(cv2)
numpy
pygame

Things you need to play:
A bright colored tube/long object you can swing around without breaking anything
A wireless mouse would be helpful as well

HOW TO BEGIN:
ensure all files are downloaded properly, and vscode has access to your laptop or desktop camera
Open gui.py and run it
start by calibrating your saber before you play any levels

CALIBRATION INSTRUCTIONS:
Click on "calibrate your saber"
Click on "calibrate lighting"
You should see a window pop up displaying what openCV sees hsv-wise.
Moving your saber around the room, click on it four times.
Then adjust the sliders. Start by moving the minimum values before adjusting the max values
hit escape once you are satisfied
Then,while standing where you plan on playing, hold the saber straight up and hit calibrate length. Keep holding until the window closes automatically.

You can use the debug button to test your calibration settings to see if it was successful

You can now hit back, and start playing a game on a selected level.

TROUBLESHOOTING:
If you find that your swing is not being detected, try swinging a bit more(not faster). The algorithm looks into how big your swing is rather than speed
Another possible reason is that the algorithm is seeing something in your background that it thinks is the saber
Note for calibration: the camera is extremely sensitive, test in a well lit room, and avoid wearing clothes that are of similar colors to your saber

If you find that you are having tracking issues, please recalibrate or try wearing a different shirt(this made a very big difference for me)

