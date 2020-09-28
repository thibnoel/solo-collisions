import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def reshapeDataToGrid(grid_map, q_steps):
    return grid_map[:,-1].reshape([q_steps[-1-i] for i in range(len(q_steps))])


def visualize2DData(data, q_ind, q_ranges, grid=False, q_steps=None, title="", cmap=plt.cm.viridis):
    if(grid):
        if(q_steps == None):
            print("arg 'q_steps' missing for grid visualization")            
        data = reshapeDataToGrid(data, q_steps)
        plt.imshow(data, extent=q_ranges[0]+q_ranges[1], cmap=cmap)
        plt.xlabel("q[{}] - {} ticks".format(q_ind[0], q_steps[0]))
        plt.ylabel("q[{}] - {} ticks".format(q_ind[1], q_steps[1]))
        plt.title(title)
    else:
        plt.scatter(data[:,0], data[:,1], c=data[:,2], cmap=cmap)
        plt.xlabel("q[{}]".format(q_ind[0]))
        plt.ylabel("q[{}]".format(q_ind[1]))
        plt.title(title)


def visualize3DData(plt_figure, data, q_ind, q_ranges, grid=False, q_steps=None, subplot_pos=111, title="", cmap=plt.cm.viridis):
    ax = plt_figure.add_subplot(subplot_pos, projection='3d')
    ax.scatter(data[:,0], data[:,1], data[:,2], c=data[:,3], cmap=cmap)#, vmin=np.min(data[:,3]))
    #ax10.set_facecolor('xkcd:grey')
    ax.set_xlabel("q[{}]".format(q_ind[0]))
    ax.set_ylabel("q[{}]".format(q_ind[1]))
    ax.set_zlabel("q[{}]".format(q_ind[2]))
    ax.set_xlim(q_ranges[0])
    ax.set_ylim(q_ranges[1])
    ax.set_zlim(q_ranges[2])
    ax.set_title(title)


def visualizeFlat3DData(data, q_ind, q_ranges, q_steps, title="", cmap=plt.cm.viridis):
    plt.title(title)
    
    nb_sq = np.sqrt(q_steps[2])
    if nb_sq % 1 > 0:
        nb_sq = int(nb_sq) + 1
    else:
        nb_sq = int(nb_sq)
    for k in range(q_steps[2]):
        plt.subplot(nb_sq,nb_sq,k+1)
        offset = q_steps[0]*q_steps[1]
        local_data = data[k*offset:(k+1)*offset]
        visualize2DData(local_data, q_ind[:2], q_ranges[:2], grid=True, q_steps=q_steps[:2], title="q[{}] = {:.2f}".format(q_ind[2], local_data[0,2]), cmap=cmap)

