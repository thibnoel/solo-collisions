#!/usr/bin/python3

import numpy as np 
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from solo_shoulder_extended_collision_sampling import *
#from collision_approx_fourier import thresholdFilter, evalApproxGradFFT
from solo_shoulder_approx_common import *

def customActivation(x):
    #return x*torch.tanh(x)
    return x*torch.sigmoid(x)
    #return x*x*x*torch.abs(x)/(1+x*x*x*x)
    #return torch.tanh(x)


class Net(nn.Module):

    def __init__(self, architecture, activation=torch.tanh):
        super(Net, self).__init__()
        # an affine operation: y = Wx + b
        #self.layers = nn.ParameterList()
        layers = []
        self.activation = activation

        layers.append(nn.Linear(architecture[0][0], architecture[0][1]))
        for k in range(len(architecture)-1):
            layers.append(nn.Linear(architecture[k+1][0], architecture[k+1][1]))
        self.layers = nn.ModuleList(layers)
        self.architecture = architecture

    def param_string(self):
        s = ""
        for k in range(len(self.architecture)):
            s = s + "[{},{}] ".format(self.architecture[k][0], self.architecture[k][1])
        return s

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


def buildData(distance2D):
    n,m = distance2D.shape
    X = []
    Y = []
    maxD = np.max(dist_field)
    for i in range(n):
        for j in range(m):
            #X.append([np.cos(2*np.pi*i/n),np.cos(2*np.pi*j/m),np.sin(2*np.pi*i/n),np.sin(2*np.pi*j/m)])
            #Y.append(distance2D[i,j])
            X.append([np.cos(2*np.pi*i/n),np.cos(2*np.pi*j/m),np.sin(2*np.pi*i/n),np.sin(2*np.pi*j/m), distance2D[i,j]])
    return np.array(X)#,np.array(Y)/maxD


def preprocessData(dist_samples):
    data = dist_samples.copy()
    dim = dist_samples.shape[1] - 1
    X = np.zeros((data.shape[0], 2*dim))
    for k in range(dim):
        X[:,k] = np.cos(data[:,k])
        X[:,dim+k] = np.sin(data[:,k])
    #X[:,2] = np.sin(data[:,0])
    #X[:,3] = np.sin(data[:,1])

    Y = data[:,dim]

    X = torch.from_numpy(X).float()
    Y = torch.from_numpy(Y).float()

    return X, Y


def inputJac(x,y):
    J = np.array([-np.sin(x),0,0,-np.sin(y), np.cos(x),0,0,np.cos(y)])
    return J.reshape(4,2)


def inputJac4(x):
    J = np.array([-x[2],0,0,-x[3],x[0],0,0,x[1]])
    return J.reshape(4,2)

def inputJacobian(x, dim):
    J = np.zeros((dim*2, dim))
    J_top = -np.diag([x[dim + i] for i in range(dim)])
    J_bottom = np.diag([x[i] for i in range(dim)])

    J[:dim,:] = J_top
    J[dim:,:] = J_bottom
    
    return J

def buildGridPred(reg, distance2D):
    n,m = distance2D.shape
    pred = np.zeros((n,m))
    jac = np.zeros((n,m,2))
    
    for i in range(n):
        pred_i = np.zeros(m)
        jac_i = np.zeros((m,2))
        for j in range(m):
            x_in = torch.from_numpy(np.array([np.cos(-np.pi + 2*np.pi*i/n),np.cos(-np.pi + 2*np.pi*j/m),np.sin(-np.pi + 2*np.pi*i/n),np.sin(-np.pi + 2*np.pi*j/m)])).view(1,-1).float()
            d_pred = reg(x_in).item()
            j_pred = reg.jacobian(x_in)
            #j_pred = torch.mm(j_pred.view(1,-1),torch.from_numpy(inputJac(2*np.pi*i/n,2*np.pi*j/m)).float())
            j_pred = torch.mm(j_pred.view(1,-1),torch.from_numpy(inputJac4(x_in.view(-1,1))).float())
            pred_i[j] = d_pred
            jac_i[j] = j_pred.data.numpy()
        pred[i,:] = pred_i
        jac[i,:] = jac_i
    return pred, jac


def buildPred(reg, ref_dist, dim=2, eval_jac=False):
    pred = []
    J = []
    for i, sample in enumerate(ref_dist):
        q = sample[:dim]
        x_in = np.concatenate((np.cos(q), np.sin(q)))
        x_in = torch.from_numpy(x_in).view(1,-1).float()
        '''
        if(dim == 2):
            q = sample[:dim]
            
            x_in = torch.from_numpy(np.array([np.cos(q[0]),np.cos(q[1]),np.sin(q[0]),np.sin(q[1])])).view(1,-1).float()
        else:
            q = sample[:3]
            x_in = torch.from_numpy(np.array([np.cos(q[0]),np.cos(q[1]),np.cos(q[2]),np.sin(q[0]),np.sin(q[1]), np.sin(q[2])])).view(1,-1).float()
        '''
        d_pred = reg(x_in).item()
        if eval_jac:
            j_pred = reg.jacobian(x_in)
            j_pred = torch.mm(j_pred.view(1,-1),torch.from_numpy(inputJacobian(x_in.view(-1,1), dim)).float())
            J.append(j_pred.data.numpy())
        res = []
        res.extend(q)
        res.append(d_pred)
        pred.append(res)
        print("<NEURAL NETWORK EVAL> Predicted {} / {} configs".format(i+1, len(ref_dist)), end='\r')
    print('')
    if eval_jac:
        return np.array(pred), np.array(J)
    else:
        return np.array(pred)


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


def trainNetSobolev(net, X, Y, Jlab, nb_epochs, batch_size):
    # Train the network
    for epoch in range(nb_epochs):  # loop over the dataset multiple times

        shuffled_ind = np.arange(0,len(X))
        np.random.shuffle(shuffled_ind)

        running_loss = 0.0
        for k in range(int(len(X)/batch_size)):
            inputs = X[shuffled_ind[k:k+batch_size]]
            labels = Y[shuffled_ind[k:k+batch_size]].view(batch_size,-1)
            Jlabels = Jlab[shuffled_ind[k:k+batch_size]].view(batch_size,-1)

            # zero the parameter gradients
            optimizer.zero_grad()
            
            # forward + backward + optimize
            outputs = net(inputs)
            Joutputs = net.jacobian(inputs)
            Jout2 = torch.empty(batch_size,2)
            Joutputs.requires_grad = True
            Jto_in = np.concatenate([np.array(inputJac4(inputs[i].view(-1,1))) for i in range(batch_size)])
            for i in range(batch_size):
                Jout2[i] = torch.mm(Joutputs[i,:].view(1,-1),torch.from_numpy(Jto_in[4*i:4*(i+1),:]).float())
            Joutputs = Jout2#*(2*np.pi)/res

            loss = criterionF(outputs, labels) + criterionJ(Joutputs, Jlabels.float())
            loss.backward()
            optimizer.step()

            # print statistics
            running_loss += loss.item()
            if k % 20 == 19:    # print every 2000 mini-batches
                print('[%d, %5d] loss: %.3f' %
                    (epoch + 1, k + 1, running_loss))
                running_loss = 0.0
    return net


def randomNetwork(nb_hidden_range, size_hidden_range, activations):
    nb_hidden_layers = np.random.randint(nb_hidden_range[0], nb_hidden_range[1])
    act_ind = np.random.randint(0, len(activations))
    curr_in = 4
    net_arch = []
    for k in range(nb_hidden_layers):
        size_layer = np.random.randint(size_hidden_range[0], size_hidden_range[1])
        if k == nb_hidden_layers -1:
            size_layer = 1
        layer_arch = [curr_in, size_layer]
        curr_in = size_layer
        net_arch.append(layer_arch)

    net = Net(net_arch, activation=activations[act_ind])
    return net, act_ind


def comparedTraining(n_networks):
    res = 100

    activations = [customActivation, torch.sigmoid, F.softsign, torch.tanh]
    activations_names=['custom','sigmoid','softsign','tanh']
    hidden_layers_range = [2,5]
    hidden_layers_size = [4,21]

    robot, rmodel, rdata, gmodel, gdata = initSolo()
    ref_config = np.zeros(robot.q0.shape)

    x_rot_range = [-np.pi, np.pi]
    y_rot_range = [-np.pi, np.pi]
    knee_rot_range = [-np.pi, np.pi]

    q_ind = [7,8]
    q_ranges = [x_rot_range, y_rot_range]
    q_steps = [400,400]

    bound2d = np.load("./npy_data/2d_bound_ref_5000samp.npy", allow_pickle=True)
    #bound3d = np.load("./npy_data/3d_bound_ref_200x200.npy", allow_pickle=True)
    bound = bound2d

    col_map_train = sampleRandomCollisionMap(ref_config, q_ind, q_ranges, 50000, [0,1], rmodel, rdata, gmodel, gdata, computeDist=False)
    col_map_test = sampleGridCollisionMap(ref_config, q_ind, q_ranges, q_steps, [0,1], rmodel, rdata, gmodel, gdata, computeDist=False)

    dist_metric='euclidean'
    data = spatialToArticular(col_map_train, bound, metric=dist_metric)
    test_data = spatialToArticular(col_map_test,bound, metric=dist_metric)

    bound_data = np.zeros((bound.shape[0], bound.shape[1]+1))
    bound_data[:,:bound.shape[1]] = bound

    dist_field = test_data

    X, Y = preprocessData(data)
    X_bound, Y_bound = preprocessData(bound_data)

    X = torch.cat((X,X_bound))
    Y = torch.cat((Y,Y_bound))

    criterionF = nn.MSELoss(reduction='sum')

    print("\n")
    for k in range(n_networks):
        net, activation = randomNetwork(hidden_layers_range, hidden_layers_size, activations)
        optimizer = optim.Adam(net.parameters(), lr=0.005)
        net = trainNet(net, X, Y, 50, 500, optimizer, criterionF, verbose=False)
        pred = buildPred(net, dist_field, dim=2)
        bin_pred_offset = binaryPredOptimOffset(dist_field, pred, optimRate=1e-3)
        bin_diff = binaryDiffMap(dist_field, pred, offset=bin_pred_offset)
        lost_space = np.count_nonzero(bin_diff[:,-1] != 0)
        print("RANDOM NETWORK {}".format(k+1))
        print("Architecture : " + net.param_string())
        print("Activation : " + activations_names[activation])
        print("Loss on grid pred : {:.2f}".format(criterionF(torch.from_numpy(pred), torch.from_numpy(dist_field))))
        print("Lost range of motion : {:.2f}% (with pred. offset d > {:.3f} )".format(100*lost_space/np.count_nonzero(dist_field > 0), bin_pred_offset))
        print('#'*50)


if __name__ == "__main__":
    #comparedTraining(10)

    # Instantiate a network
    net = Net([[4, 8],[8,1]], activation=torch.tanh)
    print(net.layers)

    #res = 100

    # Initialize data sampling parameters
    robot, rmodel, rdata, gmodel, gdata = initSolo()
    ref_config = np.zeros(robot.q0.shape)

    x_rot_range = [-np.pi, np.pi]
    y_rot_range = [-np.pi, np.pi]
    knee_rot_range = [-np.pi, np.pi]

    #2D
    
    q_ind = [7,8]
    q_ranges = [x_rot_range, y_rot_range]
    q_steps = [200,200]
    
    #3D
    '''
    q_ind = [7,8,9]
    q_ranges = [x_rot_range, y_rot_range, knee_rot_range]
    q_steps = [100,100,100]
    '''

    # Load an existing sampled boundary
    #bound3d = np.load("./npy_data/3d_bound_ref_200x200.npy", allow_pickle=True)
    bound2d = np.load("./npy_data/2d_bound_ref_5000samp.npy", allow_pickle=True)
    bound = bound2d

    # Sample a training set and a test set
    #col_map_train = sampleRandomCollisionMap(ref_config, q_ind, q_ranges, 100000, [0,1], rmodel, rdata, gmodel, gdata, computeDist=False)
    #col_map_test = sampleAroundBoundDistanceMap(ref_config, q_ind, q_ranges, 20000, bound, [0], rmodel, rdata, gmodel, gdata, computeDist=False)
    #col_map_test = sampleGridCollisionMap(ref_config, q_ind, q_ranges, q_steps, [0,1], rmodel, rdata, gmodel, gdata, computeDist=False)

    # Conversion to articular distance
    #dist_metric='euclidean'
    #train_data = spatialToArticular(col_map_train, bound, metric=dist_metric)
    #test_data = spatialToArticular(col_map_test,bound, metric=dist_metric)

    #2D
    train_data = np.load('./npy_data/datasets/2d/ref_randSampling_articularDist_100000samples.npy')
    test_data = np.load('./npy_data/datasets/2d/ref_gridSampling_articularDist_{}x{}.npy'.format(q_steps[0], q_steps[1]))

    #3D
    #train_data = np.load('./npy_data/datasets/3d/ref_randSampling_articularDist_1000000samples.npy')
    #test_data = np.load('./npy_data/datasets/3d/ref_gridSampling_articularDist_{}x{}x{}samples.npy'.format(q_steps[0], q_steps[1], q_steps[2]))

    # Modify the distance 
    #tanh_coeff = 1

    #train_data[:,-1] = np.tanh(tanh_coeff*train_data[:,-1])
    #test_data[:,-1] = np.tanh(tanh_coeff*test_data[:,-1])

    # Use the boundary points as additional data
    bound_data = np.zeros((bound.shape[0], bound.shape[1]+1))
    bound_data[:,:-1] = bound

    bound_data = bound_data[:300000]

    dist_field = test_data

    # Format data for PyTorch
    X, Y = preprocessData(train_data)
    X_bound, Y_bound = preprocessData(bound_data)

    X, Y = torch.cat((X,X_bound)), torch.cat((Y,Y_bound))

    # Preprocess J for Sobolev training
    #J = np.concatenate((Jy.reshape(-1,1), Jx.reshape(-1,1)))
    #Jtorch = torch.from_numpy(J)
    #Jtorch = Jtorch.view(-1,2)

    # Define the optimizer
    criterionF = nn.MSELoss(reduction='sum')
    criterionJ = nn.MSELoss(reduction='sum')
    Jloss_coeff = 1
    optimizer = optim.Adam(net.parameters(), lr=0.005)

    # Train the network
    net = trainNet(net, X, Y, 100, 1000, optimizer, criterionF)
    #net = trainNet(net, X_bound, Y_bound, 50, 500)

    # RESULTS
    print('Finished Training')

    pred, J = buildPred(net, test_data, eval_jac=True, dim=len(q_ind))
    #pred = buildPred(net, test_data, dim=len(q_ind))
    print("Normalized loss on grid pred : {}".format(criterionF(torch.from_numpy(pred), torch.from_numpy(test_data))/(test_data[:,-1].size)))

    #plotUnordered2DResults(dist_field, pred, title="Neural net. approx.\nArchitecture : " + net.param_string())
    plotGrid2DResults(dist_field, pred, q_ranges, q_steps, title="Neural net. approx.\nArchitecture : " + net.param_string())
    #plotUnordered3DResults(test_data, pred, title="Neural net. approx.\nArchitecture : " + net.param_string())
    
    #fig, anim_ims = plotAnimGrid3DResults(test_data, pred, q_ranges, q_steps, title="Neural net. approx.\nArchitecture : " + net.param_string(), interval=50)
    #ani = animation.ArtistAnimation(fig, anim_ims, interval=50, blit=True)
    
    J = J.reshape(q_steps[0]*q_steps[1],2)
    #J = J.reshape(q_steps[0]*q_steps[1]*q_steps[2],3)
    plot2DGridJacobian(J, q_ind, q_ranges, q_steps, title="Activation : sigmoid")
    #plot3DJacobian(test_data, J, q_ind, q_ranges, q_steps)

    '''
    pred = reshapeGridData2D(pred, q_steps)

    pred_fft = np.fft.fft2(pred)
    Jx, Jy = evalApproxGradFFT(pred_fft)
    Jx, Jy = np.fft.ifft2(Jx).real, np.fft.ifft2(Jy).real

    plt.figure()
    plt.subplot(1,2,1)
    plt.imshow(Jx, origin='lower', cmap=plt.cm.PiYG)
    plt.title("Neural net. FFT derivation (Jx)")

    plt.subplot(1,2,2)
    plt.imshow(Jy, origin='lower', cmap=plt.cm.PiYG)
    plt.title("Neural net. FFT derivation (Jy)")
    '''
    #writer = animation.ImageMagickWriter(fps=10, codec=None, bitrate=-1, extra_args=None, metadata=None)
    #ani.save("test.gif",writer=writer)
    #plt.show()

    save=True
    savepath = "./pytorch_data/test_2Dmodel_tanh_481.pth"
    if(save):
        torch.save(net.state_dict(), savepath)
        