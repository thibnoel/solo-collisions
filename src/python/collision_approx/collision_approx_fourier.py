import numpy as np

from collision_approx.collision_approx_common import *

# Approximate the collision in articular space by filtering its Fourier transform
# Implemented only for 2D inputs (ie 2 DoF)
# The neural network approach works and generalizes better, but this script is kept for reference


# Load distance data
# The target file has to contain the data formatted to a grid (see in collision_sampling.articular_space_coll_visualization to reshape sampled data)
def loadDistField(path, scale_length):
    return np.load(path, allow_pickle=True)*2*np.pi*scale_length/res


def thresholdFilter(threshold, estim):
    return (np.log(1 + abs(estim)) > threshold)*estim


def getThreshTransform(dist_field, filterThreshold, dct=False):
    raw_transform = dctn(dist_field - np.min(dist_field)) if dct else np.fft.fft2(dist_field - np.min(dist_field))
    return thresholdFilter(filterThreshold, raw_transform)


# Evaluate the distance from the FT coeffs for a given value of the inputs
def evalApproxDistFromFFT(q0, q1, fft_estim):
    dist = 0
    n,m = fft_estim.shape
    for i in range(n):
        for j in range(m):
            dist += fft_estim[i,j]*(np.cos(2*np.pi*(q1*(i-n/2)/n + q0*(j-m/2)/m)) + 1j*np.sin(2*np.pi*(q1*(i-n/2)/n + q0*(j-m/2)/m)))
    return dist/(n*m)


# Evaluate the full jacobian from the FT coeffs (ie full range of the inputs)
def evalApproxJacFromFFT(fft_estim):
    n,m = fft_estim.shape
    J_q0 = fft_estim.copy()
    J_q1 = fft_estim.copy()
    for i in range(n):
        jcoeff_q1 = 2*np.pi*1j*i/n
        if(i>n/2):
            jcoeff_q1 = 2*np.pi*1j*(i-n)/n
        for j in range(m):
            jcoeff_q0 = 2*np.pi*1j*j/m
            if(j>m/2):
                jcoeff_q0 = 2*np.pi*1j*(j-m)/m
            J_q0[i,j] = jcoeff_q0*fft_estim[i,j] 
            J_q1[i,j] = jcoeff_q1*fft_estim[i,j]
            if(i == n/2):
                J_q1[i,j] = 0
            if(j == m/2):
                J_q0[i,j] = 0
    return J_q0, J_q1