import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from solo_shoulder_extended_collision_sampling import *
from solo_shoulder_approx_common import *


# Filters the low-amplitude coefficients of the given FFT 
def thresholdFilter(threshold, fft):
    return (np.log(1 + abs(fft)) > threshold)*fft

# Evaluate the distance from the FT coeffs
def evalApproxDistFFT(q, q_steps, fft_estim):
    def updateGridInd(curr_index, k):
        if(k>=len(curr_index)):
            return curr_index
        if(curr_index[k] < q_nb_steps[k] - 1):
            curr_index[k] += 1
            return curr_index
        else:
            curr_index[k] = 0
            curr_index = updateGridInd(curr_index, k+1)
            return curr_index

    ind = [0]*len(q_steps)
    counter = 0

    dist = 0
    dim = len(q_steps)
    nb_samples = np.array([q_steps]).prod()
    while(counter < nb_samples):

        trig_arg = 0
        for k in range(dim):
            trig_arg += q[k]*(ind[k] - q_steps[k]/2)/q_steps[k]
        dist += fft_estim[ind]*( np.cos(2*np.pi*trig_arg) + 1j*np.sin(2*np.pi*trig_arg) )

        ind = updateGridInd(ind,0)
        counter += 1

    return dist/nb_samples


if __name__ == "__main__":

    # execute only if run as a script
    robot, rmodel, rdata, gmodel, gdata = initSolo()
    ref_config = np.zeros(robot.q0.shape)  
    
    x_rot_range = [-np.pi, np.pi]
    y_rot_range = [-np.pi, np.pi]
    knee_rot_range = [-np.pi, np.pi]    

    '''
    q_ind = [7,8]
    q_ranges = [x_rot_range, y_rot_range]
    q_steps = [200,200]
    '''
    q_ind = [7,8,9]
    q_ranges = [x_rot_range, y_rot_range, knee_rot_range]
    q_steps = [100,100,100]
    
    #bound3d = np.load("./npy_data/3d_bound_ref_200x200.npy", allow_pickle=True)
    bound2d = np.load("./npy_data/2d_bound_ref_5000samp.npy", allow_pickle=True)
    bound = bound2d

    #col_map = sampleGridCollisionMap(ref_config, q_ind, q_ranges, q_steps, [0,1], rmodel, rdata, gmodel, gdata, computeDist=False)
    #dist_metric = 'euclidean'
    #col_map = spatialToArticular(col_map, bound, batch_size=10, metric=dist_metric)
    col_map = np.load('./npy_data/datasets/3d/ref_gridSampling_articularDist_100x100x100samples.npy')

    grid_map = reshapeDataToGrid(col_map, q_steps)

    fft = np.fft.fftn(grid_map)

    def animPlot(fig, grid_data, subplot_pattern, cmap=plt.cm.viridis):
        #fig = plt.figure()
        ims = []
        #ims2 = []
        for i in range(q_steps[2]):
            plt.subplot(subplot_pattern)
            im = plt.imshow(grid_data[i,:,:], cmap=cmap, animated=True)
            #plt.subplot(212)
            #im2 = plt.imshow(grid_data2[i,:,:], cmap=cmap, animated=True)
            ims.append([im])
        #ani = animation.ArtistAnimation(fig, ims, interval=50, blit=True)
        return ims
        #return ani

    def fusionIms(ims_list):
        new_ims = []
        for i in range(len(ims_list[0])):
            for k in range(len(ims_list)):
                new_ims.append(ims_list[k][i])
        return new_ims

    fig1 = plt.figure()
    #fig1 = plt.subplot(2,1,1)
    ims1 = animPlot(fig1, grid_map, 211, cmap=plt.cm.RdYlGn)

    #fig2 = plt.figure()
    #fig2 = plt.subplot(2,1,2)
    ims2 = animPlot(fig1, np.abs(np.fft.fftshift(fft)),212)

    ims = fusionIms([ims1, ims2])

    ani = animation.ArtistAnimation(fig1, ims, interval=50, blit=True)
                                    #repeat_delay=1000)
    #ani2 = animation.ArtistAnimation(fig1, ims2, interval=50, blit=True)
    
    plt.show()