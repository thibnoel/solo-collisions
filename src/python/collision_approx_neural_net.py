import numpy as np 
import matplotlib.pyplot as plt
#from solo12_shoulder_collision_utils import followBoundary, colMapToDistField
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler  

# Load the collision map from file
res = 500
col_map_file = './npy_data/collision_map_centered_res{}.npy'.format(res)
col_map = np.load(col_map_file, allow_pickle=True)
col_map = col_map.T

dist_field_file = './npy_data/updated_collision_map_distance_res{}.npy'.format(res)
dist_field = np.load(dist_field_file, allow_pickle=True)

dist_field /= res

tanh_coeff = 8
dist_field = np.tanh(tanh_coeff*dist_field)

dist_field *= 2*np.pi

def buildData(distance2D):
    n,m = distance2D.shape
    X = []
    Y = []
    for i in range(n):
        for j in range(m):
            X.append([np.cos(np.pi*i/n),np.cos(np.pi*j/m),np.sin(np.pi*i/n),np.sin(np.pi*j/m)])
            #X.append([i/n,j/m])
            Y.append(distance2D[i,j])
    return X,Y


def buildPred(reg, distance2D):
    n,m = distance2D.shape
    pred = []
    for i in range(n):
        pred_i = []
        for j in range(m):
            pred_i.append(reg.predict(np.array([np.cos(np.pi*i/n),np.cos(np.pi*j/m),np.sin(np.pi*i/n),np.sin(np.pi*j/m)]).reshape(1,-1)))
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

#plt.imshow(dist_field, cmap=plt.cm.RdYlGn)
#plt.show()

X,Y = buildData(dist_field)
reg = MLPRegressor(hidden_layer_sizes=(48),verbose=True, max_iter=400, activation='relu', tol=1e-5)


#scaler = StandardScaler()
#scaler.fit(X)
#X = scaler.transform(X)  # doctest: +SKIP

reg.fit(X,Y)

pred = buildPred(reg, dist_field)
pred = pred.flatten().reshape(res,res)

wrong_pred, lost_space, pred_error = computePredictionError(pred, dist_field, 0, optim=True, optimRate=0.001)

plt.imshow(pred, cmap=plt.cm.RdYlGn)
plt.figure()
plt.imshow(dist_field, cmap=plt.cm.RdYlGn)
plt.figure()
plt.imshow(pred_error)
plt.title("Prediction errors on binary collision check\n{:.2f}% lost space".format(100*lost_space/np.count_nonzero(dist_field > 0)))
plt.show()
