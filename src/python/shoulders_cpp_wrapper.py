from ctypes import *
import numpy as np

def getShoulderCollisionsResults(q, cdll_func):
    DoubleArray4 = c_double*4
    DoubleArray3 = c_double*3
    
    x = [np.cos(q[0]), np.cos(q[1]), np.sin(q[0]), np.sin(q[1])]
    y = np.zeros(3).tolist()
    
    x = DoubleArray4(*x)
    y = DoubleArray3(*y)
    cdll_func.solo_autocollision_nn_shoulder_forward_zero(x,y)

    return y


def getDistance(collResult):
    return np.array(collResult[0])


def getJacobian(collResult):
    return np.array(collResult[1:])
