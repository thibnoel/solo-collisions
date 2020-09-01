import numpy as np 
import matplotlib.pyplot as plt
#from solo12_shoulder_collision_utils import followBoundary, colMapToDistField
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler  

def buildData(distance2D, Jx, Jy, data_weights=[1,1,1]):
    n,m = distance2D.shape
    X = []
    Y = []
    max_dist = np.max(distance2D)
    max_Jx = np.max(Jx)
    max_Jy = np.max(Jy)
    for i in range(n):
        for j in range(m):
            X.append([np.cos(2*np.pi*i/n),np.cos(2*np.pi*j/m),np.sin(2*np.pi*i/n),np.sin(2*np.pi*j/m)])
            #X.append([i/n,j/m])
            #if(Jx==None or Jy==None):
            #    Y.append(distance2D[i,j])
            #else:
            Y.append([data_weights[0]*distance2D[i,j]/max_dist, data_weights[1]*Jx[i,j]/max_Jx, data_weights[2]*Jy[i,j]/max_Jy])
    return X,Y

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

def buildPred(reg, distance2D):
    n,m = distance2D.shape
    pred = []
    for i in range(n):
        pred_i = []
        for j in range(m):
            pred_i.append(reg.predict(np.array([np.cos(2*np.pi*i/n),np.cos(2*np.pi*j/m),np.sin(2*np.pi*i/n),np.sin(2*np.pi*j/m)]).reshape(1,-1)))
            #pred_i.append(reg.predict(np.array([i/n,j/m]).reshape(1,-1)))
        pred.append(pred_i)
    return np.array(pred)


def computePredictionError(pred_dist, dist_field, offset, optim=False, optimRate=1):
    def optimOffset(wrong_pred, pred_error, offset):
        while(wrong_pred > 0):
            offset = offset + max(wrong_pred*0.0001, optimRate)
            pred_error = (pred_dist > offset) - np.asarray((dist_field > 0), dtype=float)
            wrong_pred = np.count_nonzero(pred_error > 0)
        return wrong_pred, pred_error, offset

    pred_error = (pred_dist > offset) - np.asarray((dist_field > 0), dtype=float)
    wrong_pred = np.count_nonzero(pred_error > 0)
    if(optim):
        wrong_pred, pred_error, offset = optimOffset(wrong_pred, pred_error, offset)
    lost_space = np.count_nonzero(pred_error != 0)

    return wrong_pred, lost_space, pred_error

def plot2DPeriodic(data, nPeriods):
    wideview = np.concatenate([np.concatenate([data for i in range(nPeriods)]) for i in range(nPeriods)], axis=1)
    plt.imshow(wideview, cmap=plt.cm.RdYlGn)


def thresholdFilter(threshold, estim):
    return (np.log(1 + abs(estim)) > threshold)*estim

# Load the collision map from file
res = 200
col_map_file = './npy_data/collision_map_centered_res{}.npy'.format(res)
col_map = np.load(col_map_file, allow_pickle=True)
col_map = col_map.T

dist_field_file = './npy_data/updated_collision_map_distance_res{}.npy'.format(res)
dist_field = np.load(dist_field_file, allow_pickle=True)
dist_field /= res

tanh_coeff = 12
c = 0.6
dist_field = c*dist_field + (1-c)*np.tanh(tanh_coeff*dist_field)
dist_field *= 2*np.pi

dist_fft = np.fft.fft2(dist_field - np.min(dist_field))

Jx, Jy = evalApproxGradFFT(dist_fft)
Jthreshold = 5
Jx = thresholdFilter(Jthreshold, Jx)
Jy = thresholdFilter(Jthreshold, Jy)
Jx, Jy = np.fft.ifft2(Jx).real, np.fft.ifft2(Jy).real

# Train
X,Y = buildData(dist_field, Jx=Jx, Jy=Jy, data_weights=[1,1,1])
reg = MLPRegressor(hidden_layer_sizes=(18,18,6),verbose=True, max_iter=400, activation='relu', tol=1e-5)


#scaler = StandardScaler()
#scaler.fit(X)
#X = scaler.transform(X)  # doctest: +SKIP

reg.fit(X,Y)

pred = buildPred(reg, dist_field)
pred = pred.flatten().reshape(res,res,3)

wrong_pred, lost_space, pred_error = computePredictionError(pred[:,:,0]*10*np.max(dist_field), dist_field, 0, optim=True, optimRate=0.001)

plt.figure()
plt.subplot(3,1,1)
plt.imshow(dist_field, cmap=plt.cm.RdYlGn)
#plt.figure()
plt.subplot(3,1,2)
plt.imshow(Jx, cmap=plt.cm.PiYG)
#plt.figure()
plt.subplot(3,1,3)
plt.imshow(Jy, cmap=plt.cm.PiYG)

plt.figure()
plt.subplot(3,1,1)
plt.imshow(pred[:,:,0]*10*np.max(dist_field), cmap=plt.cm.RdYlGn)
#plot2DPeriodic(pred, 3)
plt.subplot(3,1,2)
plt.imshow(pred[:,:,1]*np.max(Jx), cmap=plt.cm.PiYG)
#plot2DPeriodic(pred, 3)
plt.subplot(3,1,3)
plt.imshow(pred[:,:,2]*np.max(Jy), cmap=plt.cm.PiYG)
#plot2DPeriodic(pred, 3)

plt.figure()
plt.imshow(pred_error)
plt.title("Prediction errors on binary collision check\n{:.2f}% lost space".format(100*lost_space/np.count_nonzero(dist_field > 0)))
plt.show()
