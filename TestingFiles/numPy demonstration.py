import numpy as np
import math
#things opencv will make me do
#convert data types acro
ones=np.ones((12,12),np.uint8)
ones*=5
ones[4][2]=43
ones32=np.float32(ones)
ones32*=5.365
string=np.array(['hello','hi','test'])

#things my project will make me do:
#mostly basic linear algebra

u=np.array([1,1])
v=np.array([1,0])
angle=math.acos(np.dot(u,v)/(np.linalg.norm(u)*np.linalg.norm(v)))
angle=math.degrees(angle)#should give me 45 degrees
print(angle)
#cross product
a=np.array([1,0,0])
b=np.array([0,1,0])
c=np.cross(a,b)#should give us a vector [0,0,1] normal to a and b
print(c)
#we will be given a vector of constant magnitude, 