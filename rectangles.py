import cv2 as cv
import numpy as np


img_window= "img"
img = np.zeros((512,512,3), np.uint8)
a=(384.0,0)
b=(510,128)
blue=(255,0,0)
cv.rectangle(img,a,b,blue,3)
cv.imshow('img',img)
cv.waitKey(0)
cv.destroyAllWindows()