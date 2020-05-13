import numpy as np 
import matplotlib.pyplot as plt
from sklearn import svm
from solo12_collisions_utils import followBoundary

# Load the collision map from file
col_map_file = './npy_data/collision_map_centered_res100.npy'
col_map = np.load(col_map_file, allow_pickle=True)
traj1 = np.array(followBoundary(col_map))
traj1 = [[t[1], t[0]] for t in traj1]

traj2 = np.array(followBoundary(col_map, first_dir=2))
traj2 = [[t[1], t[0]] for t in traj2]

print(col_map.shape)
#plt.imshow(col_map.T)
#plt.show()
xSize = len(col_map[0])
ySize = len(col_map)

xx, yy = np.meshgrid(np.linspace(0, xSize, xSize),
                     np.linspace(0, ySize, ySize))

# returns neighboring indices at given dist around [k,l]
def getNeighbors(k, l, dist):
        neighbors = []
        dist = int(dist)
        for i in range(2*dist):
                for j in range(2*dist):
                        neighbors.append([k - dist + i, l - dist + j])
        return neighbors

X = []
for i in range(xSize):
        for j in range(ySize):
                neighbors = getNeighbors(i,j,2)
                append = False
                '''
                for n in neighbors:
                        if(n in traj1 or n in traj2):
                                append = True
                if(append or (i%3 == 0 and j%3 == 0)):
                        X.append([i,j])   
                '''  
                X.append([i,j])    

X = np.array(X)
print(X.shape)
Y = col_map[X[:,0],X[:,1]] > 0 #for classifier
clf = svm.NuSVC(nu=0.5)
clf.fit(X,Y)
support = np.array(clf.support_vectors_)
print("Nb. support vectors : \n{}".format(clf.n_support_))
print("Support vectors : \n{}".format(support))

# plot the decision function for each datapoint on the grid
Z = clf.decision_function(np.c_[xx.ravel(), yy.ravel()])
Z = Z.reshape(xx.shape)

plt.imshow(Z, interpolation='nearest',
           extent=(xx.min(), xx.max(), yy.min(), yy.max()), aspect='auto',
           origin='lower', cmap=plt.cm.PuOr_r)
contours = plt.contour(xx, yy, Z, levels=[0], linewidths=2,
                       linestyles='dashed', colors=['red'])
#plt.scatter(X[:, 0], X[:, 1], s=35, c=Y, cmap=plt.cm.Paired,
#            edgecolors='k')
plt.scatter(support[:,0], support[:,1], c='red', s=15)
plt.xticks(())
plt.yticks(())
plt.axis([0,xSize,0,ySize])
plt.show()