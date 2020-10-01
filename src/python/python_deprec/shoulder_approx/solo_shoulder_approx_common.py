import numpy as np 
import matplotlib.pyplot as plt 
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D


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


def binaryDiffMap(ref_dist, pred_dist, offset=0):
    ref_bin = np.asarray((ref_dist[:,-1] > 0), dtype=float)
    pred_bin = np.asarray((pred_dist[:,-1] > offset), dtype=float)
    diff_bin = ref_dist.copy()
    diff_bin[:,-1] = ref_bin - pred_bin
    return diff_bin


def binaryPredOptimOffset(ref_dist, pred_dist, optimStart=0, optimRate=1e-3):
    curr_offset = optimStart
    curr_invalid_pred = float('inf')
    while curr_invalid_pred > 0 :
        bdm = binaryDiffMap(ref_dist, pred_dist, curr_offset)
        curr_invalid_pred = np.count_nonzero(bdm[:,-1] < 0)
        curr_offset += optimRate
    return curr_offset


def distCorrelation(ref_dist, pred_dist, offset=0):
    distCorr = np.zeros((ref_dist.shape[0], 2))
    distCorr[:,0] = ref_dist[:,-1]
    distCorr[:,1] = pred_dist[:,-1]
    distCorr[:,1] -= offset
    return distCorr


def reshapeGridData2D(grid_map, q_steps):
    return grid_map.reshape(q_steps[1], q_steps[0])

def reshapeGridData3D(grid_map, q_steps):
    return grid_map.reshape(q_steps[2], q_steps[1], q_steps[0])

def reshapeDataToGrid(grid_map, q_steps):
    return grid_map[:,-1].reshape([q_steps[-1-i] for i in range(len(q_steps))])

# Evaluate the jacobian from the FT coeffs
def evalFTJac(fft_estim):
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
            estim_x[i,j] = x_grad*fft_estim[i,j] 
            estim_y[i,j] = y_grad*fft_estim[i,j] 
            if(i == n/2):
                estim_y[i,j] = 0
            if(j == m/2):
                estim_x[i,j] = 0
    return estim_x, estim_y

# Same, without *2*pi/res
def newEvalFTJac(fft_estim):
    n,m = fft_estim.shape
    estim_x = fft_estim.copy()
    estim_y = fft_estim.copy()
    for i in range(n):
        y_grad = 1j*i
        if(i>n/2):
            y_grad = 1j*(i-n)
        for j in range(m):
            x_grad = 1j*j
            if(j>m/2):
                x_grad = 1j*(j-m)
            estim_x[i,j] = x_grad*fft_estim[i,j] 
            estim_y[i,j] = y_grad*fft_estim[i,j] 
            if(i == n/2):
                estim_y[i,j] = 0
            if(j == m/2):
                estim_x[i,j] = 0
    return estim_x, estim_y


def evalJacFromFFT2D(grid_map, q_steps):
    grid_data = reshapeGridData2D(grid_map[:,-1], q_steps)
    grid_fft = np.fft.fft2(grid_data)
    Jx, Jy = evalFTJac(grid_fft)
    Jx, Jy = np.fft.ifft2(Jx).real, np.fft.ifft2(Jy).real
    return Jx, Jy


def plotUnordered2DResults(ref_dist, pred_dist, title=""):
    plt.figure()
    plt.suptitle(title, fontsize=16)
    plt.subplot(2,3,1)
    plt.scatter(ref_dist[:,0], ref_dist[:,1], marker='s', c=ref_dist[:,-1], cmap=plt.cm.RdYlGn)
    plt.axis('equal')
    plt.title("Groundtruth - testing set")
    
    plt.subplot(2,3,2)
    plt.scatter(pred_dist[:,0], pred_dist[:,1], marker='s', c=pred_dist[:,-1], cmap=plt.cm.RdYlGn)
    plt.axis('equal')
    plt.title("Prediction")

    abs_diff = np.abs(ref_dist[:,-1] - pred_dist[:,-1])
    plt.subplot(2,3,3)
    plt.scatter(ref_dist[:,0], ref_dist[:,1], marker='s', c=abs_diff)
    plt.axis('equal')
    plt.title("Abs. error : max = {:.2f}".format(np.max(abs_diff)))
    plt.colorbar()

    #plt.figure()
    plt.subplot(2,3,4)
    bin_diff0 = binaryDiffMap(ref_dist, pred_dist, offset=0)
    plt.scatter(bin_diff0[:,0], bin_diff0[:,1], marker='s', c=bin_diff0[:,2], cmap=plt.cm.gray)
    plt.axis('equal')
    plt.title("Error on binary pred.")

    plt.subplot(2,3,5)
    bin_pred_offset = binaryPredOptimOffset(ref_dist, pred_dist, optimRate=1e-3)
    bin_diff = binaryDiffMap(ref_dist, pred_dist, offset=bin_pred_offset)
    lost_space = np.count_nonzero(bin_diff[:,-1] != 0)
    plt.scatter(bin_diff[:,0], bin_diff[:,1], marker='s', c=bin_diff[:,2], cmap=plt.cm.gray, vmin=-1)
    plt.axis('equal')
    plt.title("Error on binary pred. - d > {:.2f}\n{:.2f}% false positives".format(bin_pred_offset, 100*lost_space/np.count_nonzero(ref_dist > 0)))

    plt.subplot(2,3,6)
    distcorr = distCorrelation(ref_dist, pred_dist, offset=bin_pred_offset)
    dmin, dmax = np.min(ref_dist[:,-1])-0.1, np.max(ref_dist[:,-1])+0.1
    plt.scatter(distcorr[:,0], distcorr[:,1], alpha=0.4)
    plt.plot([dmin, dmax], [dmin, dmax], '--', c='black')
    plt.xlabel("Groundtruth dist.")
    plt.ylabel("Predicted dist")

def plotGrid2DResults(ref_dist, pred_dist, q_ranges, q_steps, title=""):
    plt.figure()
    plt.suptitle(title, fontsize=16)
    plt.subplot(2,3,1)
    im0 = reshapeGridData2D(ref_dist[:,-1], q_steps)
    #plt.scatter(ref_dist[:,0], ref_dist[:,1], marker='s', c=ref_dist[:,-1], cmap=plt.cm.RdYlGn)
    plt.imshow(im0, extent=q_ranges[0]+q_ranges[1], cmap=plt.cm.RdYlGn)
    #plt.axis('equal')
    plt.title("Groundtruth - testing set")
    
    plt.subplot(2,3,2)
    im1 = reshapeGridData2D(pred_dist[:,-1], q_steps)
    #plt.scatter(pred_dist[:,0], pred_dist[:,1], marker='s', c=pred_dist[:,-1], cmap=plt.cm.RdYlGn)
    plt.imshow(im1, extent=q_ranges[0]+q_ranges[1], cmap=plt.cm.RdYlGn)
    #plt.axis('equal')
    plt.title("Prediction")

    abs_diff = np.abs(ref_dist[:,-1] - pred_dist[:,-1])
    im2 = reshapeGridData2D(abs_diff, q_steps)
    plt.subplot(2,3,3)
    #plt.scatter(ref_dist[:,0], ref_dist[:,1], marker='s', c=abs_diff)
    plt.imshow(im2, extent=q_ranges[0]+q_ranges[1])
    #plt.axis('equal')
    plt.title("Abs. error : max = {:.2f}".format(np.max(abs_diff)))
    plt.colorbar()

    #plt.figure()
    plt.subplot(2,3,4)
    bin_diff0 = binaryDiffMap(ref_dist, pred_dist, offset=0)
    im3 = reshapeGridData2D(bin_diff0[:,-1], q_steps)
    #plt.scatter(bin_diff0[:,0], bin_diff0[:,1], marker='s', c=bin_diff0[:,2], cmap=plt.cm.gray)
    plt.imshow(im3, extent=q_ranges[0]+q_ranges[1], cmap=plt.cm.gray)
    #plt.axis('equal')
    plt.title("Error on binary pred.")

    plt.subplot(2,3,5)
    bin_pred_offset = binaryPredOptimOffset(ref_dist, pred_dist, optimRate=1e-4)
    bin_diff = binaryDiffMap(ref_dist, pred_dist, offset=bin_pred_offset)
    lost_space = np.count_nonzero(bin_diff[:,-1] != 0)
    im4 = reshapeGridData2D(bin_diff[:,-1], q_steps)
    #plt.scatter(bin_diff[:,0], bin_diff[:,1], marker='s', c=bin_diff[:,2], cmap=plt.cm.gray, vmin=-1)
    plt.imshow(im4, extent=q_ranges[0]+q_ranges[1], cmap=plt.cm.gray, vmin=-1)
    #plt.axis('equal')
    plt.title("Error on binary pred. - d > {:.2f}\n{:.2f}% false positives".format(bin_pred_offset, 100*lost_space/np.count_nonzero(ref_dist > 0)))

    plt.subplot(2,3,6)
    distcorr = distCorrelation(ref_dist, pred_dist, offset=bin_pred_offset)
    dmin, dmax = np.min(ref_dist[:,-1])-0.1, np.max(ref_dist[:,-1])+0.1
    plt.scatter(distcorr[:,0], distcorr[:,1], alpha=0.4)
    plt.plot([dmin, dmax], [dmin, dmax], '--', c='black')
    plt.xlabel("Groundtruth dist.")
    plt.ylabel("Predicted dist")


def plotUnordered3DResults(ref_dist, pred_dist, title="", corrOnly=False):
    fig = plt.figure()
    fig.suptitle(title, fontsize=16)

    ax1 = fig.add_subplot(231, projection='3d')
    ax1.scatter(ref_dist[:,0], ref_dist[:,1], ref_dist[:,2], marker='s', c=ref_dist[:,-1], cmap=plt.cm.RdYlGn)
    ax1.set_title("Groundtruth - testing set")

    ax2 = fig.add_subplot(232, projection='3d')
    ax2.scatter(pred_dist[:,0], pred_dist[:,1], pred_dist[:,2], marker='s', c=pred_dist[:,-1], cmap=plt.cm.RdYlGn)
    ax2.set_title("Prediction")

    abs_diff = np.abs(ref_dist[:,-1] - pred_dist[:,-1])
    ax3 = fig.add_subplot(233, projection='3d')
    ax3.scatter(ref_dist[:,0], ref_dist[:,1], ref_dist[:,2], marker='s', c=abs_diff)
    ax3.set_title("Abs. error : max = {:.2f}".format(np.max(abs_diff)))
    print("Abs. error : max = {:.2f}".format(np.max(abs_diff)))

    ax4 = fig.add_subplot(234, projection='3d')
    bin_diff0 = binaryDiffMap(ref_dist, pred_dist, offset=0)
    bin_diff0 = bin_diff0[np.where(bin_diff0[:,-1]!=0)]
    ax4.scatter(bin_diff0[:,0], bin_diff0[:,1], bin_diff0[:,2], marker='s', c=bin_diff0[:,-1], cmap=plt.cm.gray)
    ax4.set_title("Error on binary pred.")

    ax5 = fig.add_subplot(235, projection='3d')
    bin_pred_offset = binaryPredOptimOffset(ref_dist, pred_dist, optimRate=1e-3)
    bin_diff = binaryDiffMap(ref_dist, pred_dist, offset=bin_pred_offset)
    lost_space = np.count_nonzero(bin_diff[:,-1] != 0)
    bin_diff = bin_diff[np.where(bin_diff[:,-1] !=0)]
    ax5.scatter(bin_diff[:,0], bin_diff[:,1], bin_diff[:,2], marker='s', c=bin_diff[:,2], cmap=plt.cm.rainbow)
    ax5.set_title("Error on binary pred. - d > {:.2f}\n{:.2f}% false positives".format(bin_pred_offset, 100*lost_space/np.count_nonzero(ref_dist > 0)))
    print("d > {:.2f} -> {:.3f}% false positives".format(bin_pred_offset, 100*lost_space/np.count_nonzero(ref_dist > 0)))

    ax6 = fig.add_subplot(236)
    distcorr = distCorrelation(ref_dist, pred_dist, offset=bin_pred_offset)
    dmin, dmax = np.min(ref_dist[:,-1])-0.1, np.max(ref_dist[:,-1])+0.1
    ax6.scatter(distcorr[:,0], distcorr[:,1], alpha=0.4)
    ax6.plot([dmin, dmax], [dmin, dmax], '--', c='black')
    ax6.set_xlabel("Groundtruth dist.")
    ax6.set_ylabel("Predicted dist")


def plotAnim3DGrid(data, q_steps):
    fig = plt.figure()
    ims = []
    for i in range(q_steps[2]):
        im = plt.imshow(data[i,:,:], cmap=plt.cm.RdYlGn, animated=True)
        ims.append([im])
    ani = animation.ArtistAnimation(fig, ims, interval=50, blit=True)
                                #repeat_delay=1000)


def animPlot3DGrid(fig, grid_data, q_steps, subplot_pattern, cmap=plt.cm.viridis, interval=50):
    ims = []
    for i in range(q_steps[2]):
        plt.subplot(subplot_pattern)
        im = plt.imshow(grid_data[i,:,:], cmap=cmap, animated=True)
        ims.append([im])
    #ani = animation.ArtistAnimation(fig, ims, interval=interval, blit=True)
    return ims

def fusionIms(ims_list):
    new_ims = []
    for i in range(len(ims_list[0])):
        for k in range(len(ims_list)):
            new_ims.append(ims_list[k][i])
    return new_ims



def plotAnimGrid3DResults(ref_dist, pred_dist, q_ranges, q_steps, title="", interval=50):
    fig = plt.figure(figsize=(15, 10))
    plt.suptitle(title, fontsize=16)

    #plt.subplot(2,3,1)
    im0 = reshapeGridData3D(ref_dist[:,-1], q_steps)
    #plt.scatter(ref_dist[:,0], ref_dist[:,1], marker='s', c=ref_dist[:,-1], cmap=plt.cm.RdYlGn)
    #plt.imshow(im0, extent=q_ranges[0]+q_ranges[1], cmap=plt.cm.RdYlGn)
    a0 = animPlot3DGrid(fig, im0, q_steps, 231, cmap=plt.cm.RdYlGn, interval=interval)
    #plt.axis('equal')
    plt.title("Groundtruth - testing set")
    
    #plt.subplot(2,3,2)
    im1 = reshapeGridData3D(pred_dist[:,-1], q_steps)
    #plt.scatter(pred_dist[:,0], pred_dist[:,1], marker='s', c=pred_dist[:,-1], cmap=plt.cm.RdYlGn)
    #plt.imshow(im1, extent=q_ranges[0]+q_ranges[1], cmap=plt.cm.RdYlGn)
    a1 = animPlot3DGrid(fig, im1, q_steps, 232, cmap=plt.cm.RdYlGn, interval=interval)
    #plt.axis('equal')
    plt.title("Prediction")

    abs_diff = np.abs(ref_dist[:,-1] - pred_dist[:,-1])
    im2 = reshapeGridData3D(abs_diff, q_steps)
    #plt.subplot(2,3,3)
    #plt.scatter(ref_dist[:,0], ref_dist[:,1], marker='s', c=abs_diff)
    #plt.imshow(im2, extent=q_ranges[0]+q_ranges[1])
    a2 = animPlot3DGrid(fig, im2, q_steps, 233, interval=interval)
    #plt.axis('equal')
    plt.title("Abs. error : max = {:.2f}".format(np.max(abs_diff)))
    plt.colorbar()

    #plt.figure()
    #plt.subplot(2,3,4)
    bin_diff0 = binaryDiffMap(ref_dist, pred_dist, offset=0)
    im3 = reshapeGridData3D(bin_diff0[:,-1], q_steps)
    #plt.scatter(bin_diff0[:,0], bin_diff0[:,1], marker='s', c=bin_diff0[:,2], cmap=plt.cm.gray)
    #plt.imshow(im3, extent=q_ranges[0]+q_ranges[1], cmap=plt.cm.gray)
    a3 = animPlot3DGrid(fig, im3, q_steps, 234, interval=interval, cmap=plt.cm.gray)
    #plt.axis('equal')
    plt.title("Error on binary pred.")

    plt.subplot(2,3,5)
    bin_pred_offset = binaryPredOptimOffset(ref_dist, pred_dist, optimRate=1e-4)
    bin_diff = binaryDiffMap(ref_dist, pred_dist, offset=bin_pred_offset)
    lost_space = np.count_nonzero(bin_diff[:,-1] != 0)
    im4 = reshapeGridData3D(bin_diff[:,-1], q_steps)
    #plt.scatter(bin_diff[:,0], bin_diff[:,1], marker='s', c=bin_diff[:,2], cmap=plt.cm.gray, vmin=-1)
    #plt.imshow(im4, extent=q_ranges[0]+q_ranges[1], cmap=plt.cm.gray, vmin=-1)
    a4 = animPlot3DGrid(fig, im4, q_steps, 235, interval=interval, cmap=plt.cm.gray)
    #plt.axis('equal')
    plt.title("Error on binary pred. - d > {:.2f}\n{:.2f}% false positives".format(bin_pred_offset, 100*lost_space/np.count_nonzero(ref_dist > 0)))

    plt.subplot(2,3,6)
    distcorr = distCorrelation(ref_dist, pred_dist, offset=bin_pred_offset)
    dmin, dmax = np.min(ref_dist[:,-1])-0.1, np.max(ref_dist[:,-1])+0.1
    plt.scatter(distcorr[:,0], distcorr[:,1], alpha=0.4)
    plt.plot([dmin, dmax], [dmin, dmax], '--', c='black')
    plt.xlabel("Groundtruth dist.")
    plt.ylabel("Predicted dist")

    anim_ims = fusionIms([a0,a1,a2,a3,a4])
    return fig, anim_ims