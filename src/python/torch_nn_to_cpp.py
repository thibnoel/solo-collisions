import numpy as np 
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

from solo_shoulder_approx_torch_nn import *


def test2Dnet(trainedNet, q_ind, q_ranges, q_steps, test_data):
    # Initialize data sampling parameters
    robot, rmodel, rdata, gmodel, gdata = initSolo()
    ref_config = np.zeros(robot.q0.shape)

    # Load an existing sampled boundary
    #bound3d = np.load("./npy_data/3d_bound_ref_200x200.npy", allow_pickle=True)
    bound2d = np.load("./npy_data/2d_bound_ref_5000samp.npy", allow_pickle=True)
    bound = bound2d

    # Sample a training set and a test set
    col_map_test = sampleGridCollisionMap(ref_config, q_ind, q_ranges, q_steps, [0,1], rmodel, rdata, gmodel, gdata, computeDist=False)

    # Conversion to articular distance
    dist_metric='euclidean'
    test_data = spatialToArticular(col_map_test,bound, metric=dist_metric)

    pred = buildPred(trainedNet, test_data, dim=len(q_ind))

    plotUnordered2DResults(test_data, pred, title="Neural net. approx.\nArchitecture : " + trainedNet.param_string())
    plt.show()


# Load trained model
trainedModel_path = "/home/tnoel/stage/solo-collisions/src/python/pytorch_data/test_2Dmodel_481.pth"

trainedNet = Net([[4,8],[8,1]], activation=torch.tanh)
trainedNet.load_state_dict(torch.load(trainedModel_path))
# Set model to eval mode
trainedNet.eval()

# Data params
x_rot_range = [-np.pi, np.pi]
y_rot_range = [-np.pi, np.pi]
knee_rot_range = [-np.pi, np.pi]

q_ind = [7,8]
q_ranges = [x_rot_range, y_rot_range]
q_steps = [100,100]
'''
test2Dnet(trainedNet, q_ind, q_ranges, q_steps)
'''

dim = len(q_ind)
# An example input you would normally provide to your model's forward() method.
example_q = []
for i in range(dim):
    example_q.append(q_ranges[i][0] + np.random.rand()*(q_ranges[i][1] - q_ranges[i][0]))

X = np.zeros(2*dim)
for k in range(dim):
    X[k] = np.cos(example_q[k])
    X[dim+k] = np.sin(example_q[k])

example_X = torch.from_numpy(X).float()


# Use torch.jit.trace to generate a torch.jit.ScriptModule via tracing.
#traced_script_module = torch.jit.trace(trainedNet, example_X)