import numpy as np
import matplotlib.pyplot as plt
from sklearn import svm
from functools import partial
from solo_shoulder_extended_collision_sampling import sampleRandomCollisionMap, sampleGridCollisionMap, sampleAroundBoundDistanceMap, initSolo, dichotomyBoundaryPlaneSampling, spatialToArticular
from solo_shoulder_approx_common import *

def buildGridPred(svm, distance2D):
    n,m = distance2D.shape
    pred = np.zeros((n,m))
    #jac = np.zeros((n,m,2))
    
    for i in range(n):
        pred_i = np.zeros(m)
        jac_i = np.zeros((m,2))
        for j in range(m):
            x_in = np.array([-np.pi + 2*np.pi*j/m, -np.pi + 2*np.pi*i/n]).reshape(1,-1)
            d_pred = svm.predict(x_in)[0]
            #j_pred = reg.jacobian(x_in)
            pred_i[j] = d_pred
            #jac_i[j] = j_pred.data.numpy()
        pred[i,:] = pred_i
        #jac[i,:] = jac_i
    return pred#, jac


def buildPred(svm, ref_dist):
    pred = []
    for sample in ref_dist:
        q = sample[:2]
        x_in = q.reshape(1,-1)
        d_pred = svm.predict(x_in)
        res = []
        res.extend(q)
        res.append(d_pred[0])
        pred.append(res)
    return np.array(pred)



def computeGridPredictionError(pred_dist, dist_field, offset, optim=False, optimRate=1):
    def optimOffset(wrong_pred, pred_error, offset):
        while(wrong_pred > 0):
            offset = offset + optimRate
            pred_error = (pred_dist > offset) - np.asarray((dist_field > 0), dtype=float)
            wrong_pred = np.count_nonzero(pred_error > 0)
        return wrong_pred, pred_error, offset

    pred_error = (pred_dist > offset) - np.asarray((dist_field > 0), dtype=float)
    wrong_pred = np.count_nonzero(pred_error > 0)
    if(optim):
        wrong_pred, pred_error, offset = optimOffset(wrong_pred, pred_error, offset)
    lost_space = np.count_nonzero(pred_error != 0)

    return wrong_pred, lost_space, pred_error


def proxy_kernel(X,Y,K):
    gram_matrix = np.zeros((X.shape[0], Y.shape[0]))
    for i, x in enumerate(X):
        for j, y in enumerate(Y):
            gram_matrix[i, j] = K(x, y)
    return gram_matrix
    

def custom_kernel(X, Y, sigma=1):
    return np.exp(-(X-Y).T@(X-Y)/(2*sigma*sigma))

robot, rmodel, rdata, gmodel, gdata = initSolo()
ref_config = np.zeros(robot.q0.shape)

x_rot_range = [-np.pi, np.pi]
y_rot_range = [-np.pi, np.pi]

res = 200

q_ind = [7,8]
q_ranges = [x_rot_range, y_rot_range]
q_steps = [res,res]

bound2d = np.load("./npy_data/2d_bound_ref_5000samp.npy", allow_pickle=True)
bound = bound2d
'''
#data = sampleAroundBoundDistanceMap(ref_config, q_ind, q_ranges, 20000, bound, [0], rmodel, rdata, gmodel, gdata, computeDist=False)
data = sampleRandomCollisionMap(ref_config, q_ind, q_ranges, 20000, [0], rmodel, rdata, gmodel, gdata, computeDist=False)

grid = sampleGridCollisionMap(ref_config, q_ind, q_ranges, q_steps, [0], rmodel, rdata, gmodel, gdata, computeDist=False)

grid = spatialToArticular(grid, bound)
data = spatialToArticular(data, bound)
'''

train_data = np.load('./npy_data/datasets/2d/ref_randSampling_articularDist_100000samples.npy')
test_data = np.load('./npy_data/datasets/2d/ref_gridSampling_articularDist_{}x{}.npy'.format(q_steps[0], q_steps[1]))

#grid[:,2] = np.tanh(3*grid[:,2])
#data[:,2] = np.tanh(3*data[:,2])

grid = test_data
data = train_data[:50000]

grid_data = grid[:,2].reshape(res,res)

X = data[:,:2]
Y = data[:,2]

svclassifier = svm.NuSVR(kernel='rbf', gamma=2*np.pi, nu=1e-3)
#svclassifier = svm.NuSVR(kernel=partial(proxy_kernel, K=custom_kernel))
svclassifier.fit(X, Y)
support = svclassifier.support_
print("Support : {}".format(support.size))

#pred = buildGridPred(svclassifier, np.zeros((100,100)))
pred = buildPred(svclassifier, grid)
pred_Jx, pred_Jy = evalJacFromFFT2D(pred, q_steps)

fft_Jx = np.fft.fft2(pred_Jx)
fft_Jy = np.fft.fft2(pred_Jy)

plt.figure()
plt.subplot(2,2,1)
plt.imshow(pred_Jx, extent=q_ranges[0]+q_ranges[1], cmap=plt.cm.PiYG)
plt.title("J_q{}".format(q_ind[0]))

plt.subplot(2,2,3)
plt.imshow(pred_Jy, extent=q_ranges[0]+q_ranges[1], cmap=plt.cm.PiYG)
plt.title("J_q{}".format(q_ind[1]))

plt.subplot(2,2,2)
plt.imshow(np.log(np.abs(np.fft.fftshift(fft_Jx))), extent=q_ranges[0]+q_ranges[1])
plt.title("J_q{} - FFT".format(q_ind[0]))

plt.subplot(2,2,4)
plt.imshow(np.log(np.abs(np.fft.fftshift(fft_Jy))), extent=q_ranges[0]+q_ranges[1])
plt.title("J_q{} - FFT".format(q_ind[1]))

#plotUnordered2DResults(grid, pred)
plotGrid2DResults(grid, pred, q_ranges, q_steps, title="SVM : {} support vectors".format(support.size))

#J = J.reshape(q_steps[0]*q_steps[1],2)
#plot2DGridJacobian(J, q_ind, q_ranges, q_steps)
plt.show()

#support = svclassifier.support_
'''
wrong_pred, lost_space, pred_error = computeGridPredictionError(pred, grid_data, 0, optim=True, optimRate=1e-4)
plt.figure()
#plt.imshow(pred_error, extent=x_rot_range+y_rot_range,)
plt.title("Prediction errors on binary collision check\n{:.2f}% lost space".format(100*lost_space/np.count_nonzero(grid_data > 0)))

plt.figure()
#plt.imshow(pred, extent=x_rot_range+y_rot_range, cmap=plt.cm.RdYlGn)
plt.scatter(pred[:,0], pred[:,1], c=pred[:,2], cmap=plt.cm.RdYlGn)
plt.scatter(X[support][:,0], X[support][:,1], c='blue')

plt.figure()
plt.scatter(X[:,0], X[:,1], c=Y, cmap=plt.cm.RdYlGn)
'''