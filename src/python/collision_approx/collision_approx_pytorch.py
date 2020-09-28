from collision_approx.collision_approx_common import *
from collision_sampling.articular_space_coll_visualization import *

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim


# Neural net class
class Net(nn.Module):
    def __init__(self, architecture, activation=torch.tanh):
        super(Net, self).__init__()

        layers = []
        self.activation = activation

        layers.append(nn.Linear(architecture[0][0], architecture[0][1]))
        for k in range(len(architecture)-1):
            layers.append(nn.Linear(architecture[k+1][0], architecture[k+1][1]))
        self.layers = nn.ModuleList(layers)
        self.architecture = architecture

    def print_trained_params(self):
        for name, param in self.named_parameters():
            if param.requires_grad:
                print(name, param.data)

    def forward(self, x):
        #x = x.view(-1, self.num_flat_features(x))
        for k in range(len(self.layers) - 1):
            x = self.activation(self.layers[k](x))
        x = self.layers[len(self.layers)-1](x)
        return x
    
    def jacobian(self, x):
        """
        The output of net.eval() can be more than one element
        """
        if x.dim() == 1:
            return torch.autograd.functional.jacobian(self.forward, x).squeeze()
        else:
            j = [torch.autograd.functional.jacobian(self.forward, x) for x in x]
            return torch.stack(j).squeeze()

    def num_flat_features(self, x):
        size = x.size()[1:]  # all dimensions except the batch dimension
        num_features = 1
        for s in size:
            num_features *= s
        return num_features


# Preprocess the data to account for periodic character
# The network uses the inputs x(q) = [cos(q0), ..., cos(qn), sin(q0), ... sin(qn)]
def preprocessData(dist_samples):
    data = dist_samples.copy()
    dim = dist_samples.shape[1] - 1
    X = np.zeros((data.shape[0], 2*dim))
    for k in range(dim):
        X[:,k] = np.cos(data[:,k])
        X[:,dim+k] = np.sin(data[:,k])

    Y = data[:,dim]

    X = torch.from_numpy(X).float()
    Y = torch.from_numpy(Y).float()

    return X, Y


# Compute the input Jacobian J = [dx/dq]
def inputJacobian(x, dim):
    J = np.zeros((dim*2, dim))
    J_top = -np.diag([x[dim + i] for i in range(dim)])
    J_bottom = np.diag([x[i] for i in range(dim)])

    J[:dim,:] = J_top
    J[dim:,:] = J_bottom
    
    return J


def trainNet(net, X, Y, nb_epochs, batch_size, optimizer, loss_criterion, verbose=True):
    # Train the network
    for epoch in range(nb_epochs):  # loop over the dataset multiple times

        shuffled_ind = np.arange(0,len(X))
        np.random.shuffle(shuffled_ind)

        running_loss = 0.0
        for k in range(int(len(X)/batch_size)):
            inputs = X[shuffled_ind[k:k+batch_size]]
            labels = Y[shuffled_ind[k:k+batch_size]].view(batch_size,-1)
            #Jlabels = Jlab[shuffled_ind[k:k+batch_size]].view(batch_size,-1)

            # zero the parameter gradients
            optimizer.zero_grad()
            
            # forward + backward + optimize
            outputs = net(inputs)
            loss = loss_criterion(outputs, labels)
            loss.backward()
            optimizer.step()

            # print statistics
            running_loss += loss.item()
            if k % 20 == 19 and verbose:    # print every 2000 mini-batches
                print('[%d, %5d] loss: %.3f' %
                    (epoch + 1, k + 1, running_loss))
                running_loss = 0.0
    return net


def evalNetOnConfig(net, q, eval_jac=False):
    # Preprocess input
    X_in = np.concatenate((np.cos(q), np.sin(q)))
    X_in = torch.from_numpy(X_in).view(1,-1).float()

    d_pred = net(X_in).item()
    pred = np.array([d_pred])
    
    if eval_jac:
        j_pred = net.jacobian(X_in)
        j_pred = torch.mm(j_pred.view(1,-1),torch.from_numpy(inputJacobian(X_in.view(-1,1), len(q))).float())
        j_pred = j_pred.data.numpy()
        
        pred = pred.reshape(1,1)
        pred = np.concatenate((pred, j_pred.T))
    
    return np.array(pred)


# Test data in format [nb_samples, nb_dof + 1] (last column : groundtruth dist) 
def evalNetOnTestset(net, test_data, eval_jac=False):
    pred = []
    for k in range(len(test_data)):
        q = test_data[k,:-1]
        pred.append(evalNetOnConfig(net, q, eval_jac=eval_jac))
        print("<NEURAL NETWORK EVAL> Predicted {} / {} configs".format(k+1, len(test_data)), end='\r')
    print('')

    return np.array(pred)


if __name__ == "__main__":

    q_ind = [7,8]
    q_ranges = [[-np.pi, np.pi], [-np.pi, np.pi]]
    q_steps = [200,200]

    ######## 2D data
    # Load an existing sampled boundary
    bound2d = np.load("/home/tnoel/npy_data/npy_data/npy_data/2d_bound_ref_5000samp.npy", allow_pickle=True)
    bound = bound2d

    # Load an existing dataset
    train_data = np.load('/home/tnoel/npy_data/npy_data/npy_data/datasets/2d/ref_randSampling_articularDist_100000samples.npy')
    test_data = np.load('/home/tnoel/npy_data/npy_data/npy_data/datasets/2d/ref_gridSampling_articularDist_200x200.npy')

    ######## 3D data
    '''
    # Load an existing sampled boundary
    bound2d = np.load("/home/tnoel/npy_data/npy_data/npy_data/3d_bound_ref_200x200.npy", allow_pickle=True)
    bound = bound2d

    # Load an existing dataset
    train_data = np.load('/home/tnoel/npy_data/npy_data/npy_data/datasets/3d/ref_randSampling_articularDist_1000000samples.npy')
    test_data = np.load('/home/tnoel/npy_data/npy_data/npy_data/datasets/3d/ref_gridSampling_articularDist_100x100x100samples.npy')
    '''


    # Use the boundary points as additional data : IMPROVES THE RESULTS A LOT!
    bound_data = np.zeros((bound.shape[0], bound.shape[1]+1))
    bound_data[:,:-1] = bound
    #bound_data = bound_data[:100000] # Limit number of samples in 3D (very large bound dataset)
    
    # Format data for PyTorch
    X, Y = preprocessData(train_data)
    X_bound, Y_bound = preprocessData(bound_data)

    X, Y = torch.cat((X,X_bound)), torch.cat((Y,Y_bound))

    # Instantiate a network
        # Input dimension : 2*nb_dof (ex: for SOLO, input of size 4 if shoulder only, 6 if shoulder+knee)
        # Output dimension : 1 (collision distance), but the jacobian can be evaluated as well (dim. 1*nb_dof)
    net = Net([[4, 8],[8,1]], activation=torch.tanh)
    print(net.layers)

    # Define the loss and optimizer
    criterionF = nn.MSELoss(reduction='sum')
    optimizer = optim.Adam(net.parameters(), lr=0.005)

    # Train the network
    net = trainNet(net, X, Y, 50, 1000, optimizer, criterionF)

    # RESULTS
    print('Finished Training')

    pred_col_map = evalNetOnTestset(net, test_data)

    # Results visualization
    # 2D
    binDiffMap = binaryDiffMap(test_data, pred_col_map)
    corrDiffMap = binaryDiffMap(test_data, pred_col_map, offset=binaryPredOptimOffset(test_data, pred_col_map))
    dist_correlation = distCorrelation(pred_col_map, test_data)

    lostSpace = np.count_nonzero(corrDiffMap[:,-1] != 0) / np.count_nonzero(test_data > 0)

    plt.figure()
    plt.subplot(2,2,1)
    visualize2DData(test_data, q_ind, q_ranges, grid=True, q_steps=q_steps, title="Groundtruth distance", cmap=plt.cm.RdYlGn)
    plt.subplot(2,2,2)
    visualize2DData(pred_col_map, q_ind, q_ranges, grid=True, q_steps=q_steps, title="Predicted distance", cmap=plt.cm.RdYlGn)
    plt.subplot(2,3,4)
    visualize2DData(binDiffMap, q_ind, q_ranges, grid=True, q_steps=q_steps, title="Binary error", cmap=plt.cm.gray)
    plt.subplot(2,3,5)
    visualize2DData(corrDiffMap, q_ind, q_ranges, grid=True, q_steps=q_steps, title="Binary error - no FN\n{:.1f}% lost space".format(100*lostSpace), cmap=plt.cm.gray)
    plt.subplot(2,3,6)
    plt.scatter(dist_correlation[:,0], dist_correlation[:,1])
    dmin, dmax = np.min(test_data[:,-1])-0.1, np.max(test_data[:,-1])+0.1
    plt.plot([dmin, dmax], [dmin, dmax], '--', c='black')
    plt.xlabel('Groundtruth dist')
    plt.ylabel('Predicted dist')
    plt.axis('equal')

    plt.show()