import numpy as np 
from scipy import signal
from scipy.fft import dctn, idctn
from math import floor
import matplotlib.pyplot as plt

# Load the collision map from file
res = 500
col_map_file = './npy_data/collision_map_centered_res{}.npy'.format(res)
col_map = np.load(col_map_file, allow_pickle=True)
col_map = col_map.T

def loadDistField(res, scale_length):
    dist_field_file = './npy_data/updated_collision_map_distance_res{}.npy'.format(res)
    return np.load(dist_field_file, allow_pickle=True)*2*np.pi*scale_length/res

def transformDistField(dist_field, tanh_coeff, mix_prop):
    dist_field = mix_prop*dist_field + (1-mix_prop)*np.tanh(tanh_coeff*dist_field)
    return dist_field

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


def thresholdFilter(threshold, estim):
    return (np.log(1 + abs(estim)) > threshold)*estim


def vBandFilter(bandwidth, estim):
    copy = np.fft.fftshift(estim)
    copy[:,0:bandwidth] = 0
    copy[:,len(copy) - bandwidth:len(copy)] = 0
    return np.fft.ifftshift(copy)


def computePredictionError(estim, dist_field, offset, dct=False, optim=False, optimRate=1):
    def optimOffset(wrong_pred, pred_error, offset):
        while(wrong_pred > 0):
            offset = offset + optimRate
            if(dct):
                pred_error = (idctn(estim) + np.min(dist_field) > offset) - np.asarray((dist_field > 0), dtype=float)
            else:
                pred_error = abs(np.fft.ifft2(estim).real + np.min(dist_field) > offset) - np.asarray((dist_field > 0), dtype=float)
            wrong_pred = np.count_nonzero(pred_error > 0)
        return wrong_pred, pred_error, offset

    if(dct):
        pred_error = (idctn(estim) + np.min(dist_field) > offset) - np.asarray((dist_field > 0), dtype=float)
    else:
        pred_error = abs(np.fft.ifft2(estim).real + np.min(dist_field) > offset) - np.asarray((dist_field > 0), dtype=float)
    wrong_pred = np.count_nonzero(pred_error > 0)
    if(optim):
        wrong_pred, pred_error, offset = optimOffset(wrong_pred, pred_error, offset)
    lost_space = np.count_nonzero(pred_error != 0)

    return wrong_pred, lost_space, pred_error


def plotLostSpaceVsNbCoeff(dist_field, thresholds, dct=False, label=""):
    estim = dctn(dist_field - np.min(dist_field)) if dct else np.fft.fft2(dist_field - np.min(dist_field))
    trig_calls_mult = 2 
    nb_coeffs = []
    lost_space = []

    cumul_lost = np.zeros(dist_field.shape)

    for t in thresholds:
        thresh_estim = thresholdFilter(t, estim)
        n_err, lost, error_map = computePredictionError(thresh_estim, dist_field, 0, dct=dct, optim=True, optimRate=0.001)
        nb_coeffs.append(np.count_nonzero(thresh_estim))
        lost_space.append(100*lost/np.count_nonzero(dist_field > 0))

        cumul_lost = cumul_lost - error_map

    plt.plot(trig_calls_mult*np.array(nb_coeffs), lost_space, '-+', label=label+"(DCT)" if dct else label+"(FFT)")
    plt.grid(True)
    plt.xscale("log")
    #plt.xlabel("Nb. coeffs of the FFT to evaluate")
    plt.xlabel("Nb. trig. functions calls")
    plt.ylabel("Lost range of motion (%)")

    #plt.figure()
    #plt.imshow(cumul_lost)


def getThreshTransform(dist_field, filterThreshold, dct=False):
    raw_transform = dctn(dist_field - np.min(dist_field)) if dct else np.fft.fft2(dist_field - np.min(dist_field))
    #raw_transform = dctn(dist_field) if dct else np.fft.fft2(dist_field)
    return thresholdFilter(filterThreshold, raw_transform)


def plotEstimData(dist_field, filterThreshold, dct=False):
    str_method = "DCT" if dct else "FFT"
    thresh_estim = getThreshTransform(dist_field, filterThreshold, dct=dct)

    invEstim = idctn(thresh_estim) if dct else np.fft.ifft2(thresh_estim).real
    n_err, lost, error_map = computePredictionError(thresh_estim, dist_field, 0, dct=dct, optim=True, optimRate=0.001)#, optimRate=0.001)

    plt.figure()
    # Raw distance field
    plt.subplot(2,2,1)
    plt.imshow(dist_field, cmap=plt.cm.RdYlGn)   
    plt.title('Initial distance data')

    # Estimation after threshold filtering of the raw transform
    plt.subplot(2,2,2)
    plt.imshow(invEstim, cmap=plt.cm.RdYlGn)
    plt.title("Estimated distance (i" + str_method + " of thresholded transform)")

    # Filtered transform (FFT/DCT)
    plt.subplot(2,2,3)
    log_estim = np.log(1 + abs(thresh_estim)) if dct else np.fft.fftshift(np.log(1 + abs(thresh_estim)))
    plt.imshow(log_estim)
    plt.title("Filtered " + str_method + " : log(abs(F(fx,fy))) > {}\n {} non zero coeff.".format(filterThreshold,np.count_nonzero(thresh_estim)))

    # Prediction error using the approximated distance
    plt.subplot(2,2,4)
    plt.imshow(error_map)
    plt.title("Prediction errors on binary collision check\n{:.2f}% lost space".format(100*lost/np.count_nonzero(dist_field > 0)))


def plot2DPeriodic(data, nPeriods):
    wideview = np.concatenate([np.concatenate([data for i in range(nPeriods)]) for i in range(nPeriods)], axis=1)
    plt.imshow(wideview, cmap=plt.cm.RdYlGn)

# Evaluate the distance from the FT coeffs
def evalApproxDistFFT(theta_x, theta_y, fft_estim):
    dist = 0
    n,m = fft_estim.shape
    for i in range(n):
        for j in range(m):
            dist += fft_estim[i,j]*(np.cos(2*np.pi*(theta_y*(i-n/2)/n + theta_x*(j-m/2)/m)) + 1j*np.sin(2*np.pi*(theta_y*(i-n/2)/n + theta_x*(j-m/2)/m)))
    return dist/(n*m)

def newEvalApproxDistFFT(theta_x, theta_y, fft_estim):
    n,m = fft_estim.shape
    trig_coeff = np.array([[np.cos(2*np.pi*(theta_y*(i-n/2)/n + theta_x*(j-m/2)/m)) + 1j*np.sin(2*np.pi*(theta_y*(i-n/2)/n + theta_x*(j-m/2)/m)) for j in range(m)] for i in range(n)])
    return np.multiply(trig_coeff, fft_estim).sum().sum()/(n*m)



# Evaluate the distance from the FT coeffs
def evalApproxGradFFT(fft_estim):
    n,m = fft_estim.shape
    estim_x = fft_estim.copy()
    estim_y = fft_estim.copy()
    for i in range(n):
        y_grad = 2*np.pi*1j*i/n
        if(i>n/2):
            y_grad = 2*np.pi*1j*(i-n)/n
        for j in range(m):
            x_grad = 2*np.pi*1j*j/m
            if(j>m/2):
                x_grad = 2*np.pi*1j*(j-m)/m
            estim_x[i,j] = x_grad*fft_estim[i,j] #*(np.cos(2*np.pi*(theta_y*(i-n/2)/n + theta_x*(j-m/2)/m)) + 1j*np.sin(2*np.pi*(theta_y*(i-n/2)/n + theta_x*(j-m/2)/m)))
            estim_y[i,j] = y_grad*fft_estim[i,j] #*(np.cos(2*np.pi*(theta_y*(i-n/2)/n + theta_x*(j-m/2)/m)) + 1j*np.sin(2*np.pi*(theta_y*(i-n/2)/n + theta_x*(j-m/2)/m)))
            if(i == n/2):
                estim_y[i,j] = 0
            if(j == m/2):
                estim_x[i,j] = 0
    return estim_x, estim_y

# Evaluates the distance from the FT coeffs in the jacobian direction, ie a gradient descent step
# returns dist_next = dist_q + J@dq
def evalJacDescent(theta_x, theta_y, fft_estim, Jx, Jy, dq):
    dist = evalApproxDistFFT(theta_x, theta_y, np.fft.fftshift(fft_estim))
    print(dist)
    J = np.array([evalApproxDistFFT(theta_x, theta_y, np.fft.fftshift(Jx)).real, evalApproxDistFFT(theta_x, theta_y, np.fft.fftshift(Jy)).real])
    return J.T@np.array(dq) + dist

# Get a step dq along the Jacobian to go in the direction of higher distance
def getEscapeMove(theta_x, theta_y, fft_estim, Jx, Jy, mag):
    dist = newEvalApproxDistFFT(theta_x, theta_y, np.fft.fftshift(fft_estim))
    J = np.array([newEvalApproxDistFFT(theta_x, theta_y, np.fft.fftshift(Jx)).real, newEvalApproxDistFFT(theta_x, theta_y, np.fft.fftshift(Jy)).real])
    J /= np.max(np.abs(J))
    return np.array(J*mag)

# Evaluate the next 'virtual' collision encountered when going down the jacobian
def getNextJacColl(theta_x, theta_y, fft_estim, Jx, Jy):
    dist = newEvalApproxDistFFT(theta_x, theta_y, np.fft.fftshift(fft_estim)).real
    J = np.array([[newEvalApproxDistFFT(theta_x, theta_y, np.fft.fftshift(Jx)).real, newEvalApproxDistFFT(theta_x, theta_y, np.fft.fftshift(Jy)).real]])
    dq = np.linalg.pinv(J).dot(dist)
    return dq

def evalApproxDistDCT(theta_x, theta_y, dct_estim):
    dist = dct_estim[0,0]
    n,m = dct_estim.shape
    for i in range(n):
        for j in range(m):
            if not(j==0 and i==0):
                dist += 2*dct_estim[i,j]*np.cos(np.pi*i*(2*theta_y+1)/(2*n))*np.cos(np.pi*j*(2*theta_x+1)/(2*m))
    return dist/(4*n*m)

if __name__ == "__main__":
    carac_length = 0.2
    dist_field = loadDistField(res, carac_length)

    thresh = -0.1
    dist_field = (dist_field > thresh)*dist_field + (dist_field<=thresh)*thresh

    tanh_coeff = 8
    prop = 0.4
    dist_field = transformDistField(dist_field, tanh_coeff, prop)

    ap_dist_field_0 = transformDistField(dist_field, 8, 0.1)
    ap_dist_field_1 = transformDistField(dist_field, 100, 0.95)

    #dist_field = ap_dist_field_0

    threshold = 7
    #threshold = 0

    Jthreshold = 2
    Jthreshold = 0

    fft_estim = getThreshTransform(dist_field, threshold, dct=False)
    dct_estim = getThreshTransform(dist_field, threshold, dct=True)
    plotEstimData(dist_field, threshold, dct=False)

    Jx, Jy = evalApproxGradFFT(fft_estim)

    Jx = thresholdFilter(Jthreshold, Jx)
    Jy = thresholdFilter(Jthreshold, Jy)

    # Save filtered FFT coeffs to file for code generation
    #np.savetxt("fft_estim_real.csv", fft_estim.real, delimiter=',')
    #np.savetxt("fft_estim_imag.csv", fft_estim.imag, delimiter=',')

    theta_x = 0
    theta_y = np.pi + 0.5

    # x : [-Pi,Pi] -> [0,res]
    # y : [0,2*Pi] -> [0,res]
    arg_x = (theta_x + np.pi)*res/(2*np.pi)
    arg_y = theta_y*res/(2*np.pi)

    arg_x = 150
    arg_y = 250

    #theta_x = 250
    #theta_y = 250

    print("Eval FFT: approx({:.2f},{:.2f}) = {}".format(arg_x, arg_y, evalApproxDistFFT(arg_x, arg_y, np.fft.fftshift(fft_estim))))
    print("Closest true value : {}".format(np.fft.ifft2(fft_estim)[int(arg_y), int(arg_x)]))
    print("Jac x FFT: approx({:.2f},{:.2f}) = {}".format(arg_x, arg_y, evalApproxDistFFT(arg_x, arg_y, np.fft.fftshift(Jx)).real))
    print("Jac y FFT: approx({:.2f},{:.2f}) = {}".format(arg_x, arg_y, evalApproxDistFFT(arg_x, arg_y, np.fft.fftshift(Jy)).real))

    #print("Jx : {}".format(np.abs(evalApproxDistFFT(arg_x, arg_y, np.fft.fftshift(2*np.pi*arg_x*1j*fft_estim)))))
    #print("Jy : {}".format(np.abs(evalApproxDistFFT(arg_x, arg_y, np.fft.fftshift(2*np.pi*arg_y*1j*fft_estim)))))

    plt.figure()
    plt.imshow(np.fft.ifft2(fft_estim).real, cmap=plt.cm.RdYlGn)
    # Grad. descent example
    '''
    m = [arg_x, arg_y]
    lx = []
    ly = []
    for i in range(10):
        lx.append(m[0])
        ly.append(m[1])
        print(m)
        m = m + getEscapeMove(m[0], m[1], fft_estim, Jx, Jy, 10)
    plt.plot(lx, ly, 'b-+')
    '''
    next_q = np.array([arg_x, arg_y]) + getNextJacColl(arg_x, arg_y, fft_estim, Jx, Jy)[0]%(2*np.pi)
    plt.plot(arg_x, arg_y, 'g-o')
    plt.plot(next_q[0], next_q[1], 'b-o')

    plt.figure()
    plt.subplot(2,2,1)
    plt.imshow(np.fft.ifft2(Jx).real, cmap=plt.cm.PiYG, vmin = -0.01, vmax = 0.01)
    plt.title("Jx")
    plt.subplot(2,2,2)
    plt.imshow(np.fft.fftshift(np.log(1 + abs(Jx))))
    plt.title("Jx fft ({} >0)".format(np.count_nonzero(abs(Jx))))
    plt.subplot(2,2,3)
    plt.imshow(np.fft.ifft2(Jy).real, cmap=plt.cm.PiYG, vmin = -0.01, vmax = 0.01)
    plt.title("Jy")
    plt.subplot(2,2,4)
    plt.imshow(np.fft.fftshift(np.log(1 + abs(Jy))))
    plt.title("Jy fft ({} >0)".format(np.count_nonzero(abs(Jy))))

    #print("\nEval DCT: approx({:.2f},{:.2f}) = {:.2f}".format(theta_x, theta_y, evalApproxDistDCT(theta_x, theta_y, dct_estim)))
    #print("Closest true value : {}".format(idctn(dct_estim)[floor(theta_y), floor(theta_x)]))
    """
    log_threshold = 12
    #thresh_estim = vBandFilter(240,estim)
    thresh_estim = thresholdFilter(log_threshold, dct_estim)
    """
    '''
    # Diff map
    plt.figure()
    diff_map = abs(np.fft.ifft2(thresh_estim)) - dist_field + np.min(dist_field)
    plt.imshow(diff_map)
    '''

    #plt.figure()
    #plot2DPeriodic(abs(np.fft.ifft2(thresh_estim)),3)
    #plot2DPeriodic(dist_field,3)
    '''
    plt.figure()
    plotLostSpaceVsNbCoeff(dist_field, [0,5,7,7.5,8,8.5,9,9.5,10], dct=False, label="Initial dist.")
    plotLostSpaceVsNbCoeff(ap_dist_field_0, [0,5,7,7.5,8,8.5,9,9.5,10], dct=False, label="Modif. 0")
    plotLostSpaceVsNbCoeff(ap_dist_field_1, [0,5,7,7.5,8,8.5,9,9.5,10], dct=False, label="Modif. 1")
    plt.legend()
    '''
    plt.show()


