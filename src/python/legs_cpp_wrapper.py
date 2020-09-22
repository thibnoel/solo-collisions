from ctypes import *
import numpy as np


# Init C types and store function call in a vector y of dim (nb_motors + 1)*nb_pairs
def getLegsCollisionsResults(q, cdll_func, nb_motors, nb_pairs):
    ny = (nb_motors+1)*nb_pairs

    DoubleArrayIn = c_double*nb_motors
    DoubleArrayOut = c_double*ny

    y = np.zeros(ny).tolist()

    q = DoubleArrayIn(*q)
    y = DoubleArrayOut(*y)
    cdll_func.solo_autocollision_legs_legs_forward_zero(q, y)

    return y


# Extract distances from results vector
def getLegsDistances(legsCollResults, nb_motors, nb_pairs):
    return np.array([legsCollResults[i*(1+nb_motors)] for i in range(nb_pairs)])


# Extract jacobians from results vector
def getLegsJacobians(legsCollResults, nb_motors, nb_pairs):
    return np.vstack([legsCollResults[i*(1+nb_motors) + 1 : (i+1)*(1+nb_motors)] for i in range(nb_pairs)])
