import numpy as np 
import matplotlib.pyplot as plt 

# Compute the binary error map between a groundtruth coll. field and a predicted coll. field 
def binaryDiffMap(ref_dist, pred_dist, offset=0):
    ref_bin = np.asarray((ref_dist[:,-1] > 0), dtype=float)
    pred_bin = np.asarray((pred_dist[:,-1] > offset), dtype=float)
    diff_bin = ref_dist.copy()
    diff_bin[:,-1] = ref_bin - pred_bin
    return diff_bin


# Compute the optimal prediction offset t so that for a prediction method p,
# using p'(q) = p(q) - t as a new prediction method never returns any false negatives
# (i.e no potential collision is missed)
def binaryPredOptimOffset(ref_dist, pred_dist, optimStart=0, optimRate=1e-3):
    curr_offset = optimStart
    curr_invalid_pred = float('inf')
    while curr_invalid_pred > 0 :
        bdm = binaryDiffMap(ref_dist, pred_dist, curr_offset)
        curr_invalid_pred = np.count_nonzero(bdm[:,-1] < 0)
        curr_offset += optimRate
    return curr_offset


# Used to plot the predicted distances vs. groudtruth distances
def distCorrelation(ref_dist, pred_dist, offset=0):
    distCorr = np.zeros((ref_dist.shape[0], 2))
    distCorr[:,0] = ref_dist[:,-1]
    distCorr[:,1] = pred_dist[:,-1]
    distCorr[:,1] -= offset
    return distCorr


def pred_RMSE(ref_dist, pred_dist, offset=0):
    rmse = np.sqrt((ref_dist - pred_dist)**2)
    return rmse