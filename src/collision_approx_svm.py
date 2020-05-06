import numpy as np 
import matplotlib.pyplot as plt
from sklearn import svm

# Load the collision map from file
col_map_file = './collision_map_centered_res100.npy'
col_map = np.load(col_map_file, allow_pickle=True)

print(col_map.shape)
plt.imshow(col_map.T)
#plt.show()
xSize = len(col_map[0])
ySize = len(col_map)

xx, yy = np.meshgrid(np.linspace(0, xSize, xSize),
                     np.linspace(0, ySize, ySize))

X = []
for i in range(xSize):
    for j in range(ySize):
        X.append([i,j])
X = np.array(X)
Y = col_map[X[:,0],X[:,1]] > 0 #for classifier
clf = svm.NuSVC(nu=0.5)
clf.fit(X,Y)

print("Nb. support vectors : \n{}".format(clf.n_support_))
print("Support vectors : \n{}".format(clf.support_vectors_))

# plot the decision function for each datapoint on the grid
Z = clf.decision_function(np.c_[xx.ravel(), yy.ravel()])
Z = Z.reshape(xx.shape)

#plt.imshow(Z, interpolation='nearest',
#           extent=(xx.min(), xx.max(), yy.min(), yy.max()), aspect='auto',
#           origin='lower', cmap=plt.cm.PuOr_r)
contours = plt.contour(xx, yy, Z, levels=[0], linewidths=2,
                       linestyles='dashed', colors=['red'])
#plt.scatter(X[:, 0], X[:, 1], s=30, c=Y, cmap=plt.cm.Paired,
#            edgecolors='k')
plt.xticks(())
plt.yticks(())
plt.axis([0,xSize,0,ySize])
plt.show()