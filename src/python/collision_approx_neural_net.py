import numpy as np 
import matplotlib.pyplot as plt
from solo12_shoulder_collision_utils import followBoundary, colMapToDistField
from sklearn.neural_network import MLPRegressor

# Load the collision map from file
res = 500
col_map_file = './npy_data/collision_map_centered_res{}.npy'.format(res)
col_map = np.load(col_map_file, allow_pickle=True)
col_map = col_map.T

dist_field_file = './npy_data/updated_collision_map_distance_res{}.npy'.format(res)
dist_field = np.load(dist_field_file, allow_pickle=True)


def buildData(distance2D):
    n,m = distance2D.shape
    X = []
    Y = []
    for i in range(n):
        for j in range(m):
            X.append([i,j])
            Y.append(distance2D[i,j])
    return X,Y


def buildPred(reg, distance2D):
    n,m = distance2D.shape
    pred = []
    for i in range(n):
        pred_i = []
        for j in range(m):
            pred_i.append(reg.predict(np.array([i,j]).reshape(1,-1)))
        pred.append(pred_i)
    return np.array(pred)

X,Y = buildData(dist_field)
reg = MLPRegressor(hidden_layer_sizes=(12,6),verbose=True)
reg.fit(X,Y)

pred = buildPred(reg, dist_field)

plt.imshow(pred, cmap=plt.cm.RdYlGn)
plt.figure()
plt.imshow(dist_field, cmap=plt.cm.RdYlGn)
plt.show()
