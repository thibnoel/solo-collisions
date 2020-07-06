import numpy as np 
import matplotlib.pyplot as plt
import pinocchio as pio

# assumes xP, yP, zP in rect. oriented frame i.e. top left corner at [0,0,0], z-axis normal
def getPointRectDist(se3_rfMpf, xP, yP, zP, width, length):

    p = np.array([xP,yP,zP])
    pR = se3_rfMpf.act(p)

    closest_x = min(max(0,pR[0]),length)
    closest_y = min(max(0,pR[1]),width)

    closest_pR = np.array([closest_x, closest_y, 0])
    return np.linalg.norm(closest_pR - pR)

pMr = np.array([[1,0,0,0],
                [0,1,0,0],
                [0,0,1,0],
                [0,0,0,1]])
pMr = pio.SE3(pMr)

width = 1
length = 2

xP, yP, zP = -0.5,0.5,0.5

print(getPointRectDist(pMr, xP, yP, zP, width, length))