from ctypes import *
import numpy as np


# Init C types and store function call in a vector y of dim q_dim+1
def getShoulderCollisionsResults(q, cdll_func, q_dim):
    DoubleArrayIn = c_double*(2*q_dim)
    DoubleArrayOut = c_double*(1 + q_dim)

    x = np.concatenate((np.cos(q), np.sin(q))).tolist()
    y = np.zeros(1 + q_dim).tolist()
    
    x = DoubleArrayIn(*x)
    y = DoubleArrayOut(*y)
    cdll_func.solo_autocollision_nn_shoulder_forward_zero(x,y)

    return y


def getShoulderDistance(shoulderCollResult):
    return np.array(shoulderCollResult[0])


def getShoulderJacobian(shoulderCollResult):
    return np.array(shoulderCollResult[1:])
