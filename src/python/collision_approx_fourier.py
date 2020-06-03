import numpy as np 
from scipy import signal
import matplotlib.pyplot as plt
from solo12_shoulder_collision_utils import followBoundary, colMapToDistField

# Load the collision map from file
res = 200
col_map_file = './npy_data/collision_map_centered_res{}.npy'.format(res)
dist_field_file = './npy_data/updated_collision_map_distance_res{}.npy'.format(res)
col_map = np.load(col_map_file, allow_pickle=True)
col_map = col_map.T
dist_field = np.load(dist_field_file, allow_pickle=True)

"""
traj1 = followBoundary(col_map.T)
traj2 = followBoundary(col_map.T, first_dir=2)
traj2 = [traj2[-i] for i in range(len(traj2))]

traj1X = np.array([t[0] for t in traj1])
traj1Y = np.array([t[1] for t in traj1])

traj2X = np.array([t[0] for t in traj2])
traj2Y = np.array([t[1] for t in traj2])
#traj2X = np.concatenate([traj2X, traj2X + len(traj2X), traj2X + 2*len(traj2X)])
#traj2Y = np.array(3*[t[1] for t in traj2])

def approxFourier(trajX, trajY, Nh, plot=True):
    complexTraj = np.array(trajX + 1j*trajY)
    period = len(complexTraj)
    time = np.array([i for i in range(period)])

    def cn(n):
        c = complexTraj*np.exp(-1j*2*n*np.pi*time/period)
        return c.sum()/c.size

    def f(x, Nh):
        f = np.array([cn(i)*np.exp(1j*2*i*np.pi*x/period) for i in range(-Nh-1,Nh+1)])
        return f.sum()

    traj_est = np.array([f(t,Nh) for t in time])
    #plt.figure()
    if(plot):
        plt.plot(complexTraj.real - complexTraj.real[int(len(complexTraj)/3)], complexTraj.imag, 
                                                                linewidth=2, 
                                                                c='limegreen')
        plt.plot(traj_est.real[10:-10] - complexTraj.real[int(len(complexTraj)/3)], traj_est.imag[10:-10], 'r-', linewidth=2,
                                                                label="Fourier series - {} harmonics".format(Nh))
        #plt.plot(trajX, trajY_est)
    return traj_est

def approxPolynom(trajX, trajY, deg, plot=True):
    polynCoeffs = np.polyfit(trajX, trajY, deg)
    polynEval = np.poly1d(polynCoeffs)

    if(plot):
        #plt.figure()
        #plt.plot(trajX, trajY)
        plt.plot(trajX, polynEval(trajX),linewidth=2, c='yellow',label="Polynom - deg. {}".format(deg))
    return [trajX, polynEval(trajX)]

def newApproxFourier(colMap, Nh):
    period = [len(colMap), len(colMap[0])]
    xRange = np.linspace(0,period[0],period[0])
    yRange = np.linspace(0,period[1],period[1])
    gridX, gridY = np.meshgrid(xRange,yRange)

    # Compute the 2D fourier coeff of index (i,j)
    def c_mn(m,n):
        c = (1./(4*np.pi*np.pi))*colMap*np.exp(-1j*2*np.pi*m*gridX/period[0])*np.exp(-1j*2*np.pi*n*gridY/period[1])
        return c.sum()/c.size

    # Evaluate f based on the coeffs
    def f(x,y,Nh):
        f = np.array([ [c_mn(k,l)*np.exp(1j*2*np.pi*l*x/period[0])*np.exp(1j*2*np.pi*k*y/period[1]) for l in range(-Nh-1, Nh+1)] for k in range(-Nh-1, Nh+1)])
        return f.sum()

    estim = [[f(x, y, Nh) for y in yRange] for x in xRange]
    return estim

"""
"""
#print(traj2)
#plt.subplot(2,2,1)
#approxFourier(traj1, 50, plot=False)
#plt.subplot(2,2,2)
#approxPolynom(traj1, 10, plot=False)
#plt.subplot(2,2,3)
plt.figure()
plt.imshow(col_map)
plt.plot(traj1X, traj1Y, 'r')

#polynTraj1 = approxPolynom(traj1X, traj1Y, 101, plot=True)
#traj1X = np.concatenate([traj1X, traj1X + len(traj1X), traj1X + 2*len(traj1X)])
#traj1Y = np.array(3*[t[1] for t in traj1])
#fourierTraj1 = approxFourier(traj1X, traj1Y, 10, plot=True)
#fourierTraj2 = approxFourier(traj2X, traj2Y, 100, plot=True)
#plt.subplot(2,2,4)

#polynTraj2 = approxPolynom(traj2, 10, plot=True)
plt.legend()
plt.title('Collision boundary approximation')

plt.figure()
plt.grid(True)
plt.plot(traj1X, traj1Y + 2*len(col_map)/4)
#plt.plot(fourierTraj1.real[10:-10], fourierTraj1.imag[10:-10] + len(col_map)/4)
#plt.plot(polynTraj1[0] + fourierTraj1.real[int(len(fourierTraj1)/3)], polynTraj1[1])
#plt.plot(polynTraj1[0] , polynTraj1[1])
plt.show()
'''
'''
dist_field = np.array(colMapToDistField(col_map.T))
np.save('./npy_data/collision_map_distance_res1000', dist_field)

plt.figure()
plt.imshow(col_map.T)
plt.plot(traj1X, traj1Y,'r')
plt.plot(traj2X, traj2Y,'r')

plt.figure()
plt.imshow(dist_field, cmap=plt.cm.RdYlGn)
#plt.plot(traj1X, traj1Y, 'r')
#plt.plot(traj2X, traj2Y,'r')
plt.colorbar(label='Dist. to boundary')

plt.figure()
#cumul_dist_field = (dist_field > 0).astype(float) + (dist_field > 10) + (dist_field > 20) + (dist_field > 30) + (dist_field > 40)
cumul_dist_field = (dist_field > 0.1).astype(float) + (dist_field < -0.1)
plt.imshow(cumul_dist_field)

plt.show()
"""


def thresholdFilter(threshold, fft):
    return (np.log(1 + abs(fft)) > threshold)*fft

def vBandFilter(bandwidth, fft):
    copy = np.fft.fftshift(fft)
    copy[:,0:bandwidth] = 0
    copy[:,len(copy) - bandwidth:len(copy)] = 0
    return np.fft.ifftshift(copy)

def computePredictionError(fft, dist_field, offset, optim=False, optimRate=1):
    def optimOffset(wrong_pred, pred_error, offset):
        while(wrong_pred > 0):
            offset = offset + max(wrong_pred*0.001, optimRate)
            pred_error = abs(np.fft.ifft2(fft) + np.min(dist_field) > offset) - np.asarray((dist_field > 0), dtype=float)
            wrong_pred = np.count_nonzero(pred_error > 0)
        return wrong_pred, pred_error, offset

    pred_error = abs(np.fft.ifft2(fft) + np.min(dist_field) > offset) - np.asarray((dist_field > 0), dtype=float)
    wrong_pred = np.count_nonzero(pred_error > 0)
    if(optim):
        wrong_pred, pred_error, offset = optimOffset(wrong_pred, pred_error, offset)
    lost_space = np.count_nonzero(pred_error != 0)

    return wrong_pred, lost_space, pred_error

def plotLostSpaceVsNbCoeff(fft, dist_field, thresholds):
    nb_coeffs = []
    lost_space = []

    cumul_lost = np.zeros(dist_field.shape)

    for t in thresholds:
        thresh_estim = thresholdFilter(t, fft)
        n_err, lost, error_map = computePredictionError(thresh_estim, dist_field, 5, optim=True, optimRate=0.05)
        nb_coeffs.append(np.count_nonzero(thresh_estim))
        lost_space.append(100*lost/np.count_nonzero(dist_field > 0))

        cumul_lost = cumul_lost - error_map

    plt.plot(nb_coeffs, lost_space, '-+')
    plt.grid(True)
    plt.xscale("log")
    plt.xlabel("Nb. coeffs of the FFT to evaluate")
    plt.ylabel("Lost range of motion (%)")

    plt.figure()
    plt.imshow(cumul_lost)

estim = np.fft.fft2(dist_field - np.min(dist_field))

log_threshold = 12
#thresh_estim = vBandFilter(240,estim)
thresh_estim = thresholdFilter(log_threshold, estim)

plt.figure()
plt.subplot(2,2,1)
#plt.imshow(abs(np.fft.ifft2(estim)), cmap=plt.cm.RdYlGn)
plt.imshow(dist_field, cmap=plt.cm.RdYlGn)

#plt.figure()
plt.subplot(2,2,2)
plt.imshow(abs(np.fft.ifft2(thresh_estim)), cmap=plt.cm.RdYlGn)
plt.title("Estimated distance (iFFT of thresholded transform)")

#error_map = (abs(np.fft.ifft2(thresh_estim)) + np.min(dist_field) > 0) - np.asarray((dist_field > 0), dtype=float)
n_err, lost, error_map = computePredictionError(thresh_estim, dist_field, 5, optim=True)
#plt.figure()
plt.subplot(2,2,4)
plt.imshow(error_map)
plt.title("Prediction errors on binary collision check\n{:.2f}% lost space".format(100*lost/np.count_nonzero(dist_field > 0)))

#plt.figure()
plt.subplot(2,2,3)
plt.imshow(np.fft.fftshift(np.log(1 + abs(thresh_estim))))
plt.title("Filtered FFT : log(abs(F(fx,fy))) > {}\n {} non zero coeff.".format(log_threshold,np.count_nonzero(thresh_estim)))

'''
# Diff map
plt.figure()
diff_map = abs(np.fft.ifft2(thresh_estim)) - dist_field + np.min(dist_field)
plt.imshow(diff_map)
'''

print("FFT non zero values : ")
print(np.count_nonzero(thresh_estim))

print("Error ratio : ")
print(np.count_nonzero(error_map)/error_map.size)

'''
# Periodic view
plt.figure()
wideview = np.concatenate([np.concatenate([abs(np.fft.ifft2(thresh_estim)),abs(np.fft.ifft2(thresh_estim))]),
                          np.concatenate([abs(np.fft.ifft2(thresh_estim)),abs(np.fft.ifft2(thresh_estim))])], axis=1)
plt.imshow(wideview, cmap=plt.cm.RdYlGn)
'''

plt.figure()
plotLostSpaceVsNbCoeff(estim, dist_field, [0,5,8,10,11,12,12.5,13,13.5,14,14.5,15,15.5,16])
#plotLostSpaceVsNbCoeff(estim, dist_field, [0,5,10,15])

plt.show()


