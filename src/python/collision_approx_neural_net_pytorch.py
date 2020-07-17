import numpy as np 
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

class Net(nn.Module):

    def __init__(self):
        super(Net, self).__init__()
        # an affine operation: y = Wx + b
        self.fc1 = nn.Linear(4, 18)
        self.fc2 = nn.Linear(18, 12)
        self.fc3 = nn.Linear(12, 12)
        self.fc4 = nn.Linear(12, 1)

    def forward(self, x):
        x = x.view(-1, self.num_flat_features(x))
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = self.fc4(x)
        return x

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
    max_dist = np.max(distance2D)
    for i in range(n):
        for j in range(m):
            X.append([np.cos(np.pi*i/n),np.cos(np.pi*j/m),np.sin(np.pi*i/n),np.sin(np.pi*j/m)])
            Y.append(distance2D[i,j]/max_dist)
    return X,Y

def buildPred(reg, distance2D):
    n,m = distance2D.shape
    pred = []
    for i in range(n):
        pred_i = []
        for j in range(m):
            pred_i.append(reg(torch.from_numpy(np.array([np.cos(np.pi*i/n),np.cos(np.pi*j/m),np.sin(np.pi*i/n),np.sin(np.pi*j/m)]).reshape(1,-1)).float()).item())
            #pred_i.append(reg.predict(np.array([i/n,j/m]).reshape(1,-1)))
        pred.append(pred_i)
    return np.array(pred)

# Instantiate a network
net = Net()

# Load the collision map from file
res = 200
col_map_file = './npy_data/collision_map_centered_res{}.npy'.format(res)
col_map = np.load(col_map_file, allow_pickle=True)
col_map = col_map.T

dist_field_file = './npy_data/updated_collision_map_distance_res{}.npy'.format(res)
dist_field = np.load(dist_field_file, allow_pickle=True)

dist_field /= res

X,Y = buildData(dist_field)
X,Y = np.array(X), np.array(Y)
X,Y = torch.from_numpy(X).float(), torch.from_numpy(Y).float()

# Define the optimizer
criterion = nn.MSELoss()
optimizer = optim.SGD(net.parameters(), lr=0.005, momentum=0.9)

# Train the network
minibatch_size = 500
for epoch in range(500):  # loop over the dataset multiple times

    running_loss = 0.0
    for k in range(int(len(X)/minibatch_size)):
        inputs = X[k:k+minibatch_size]
        labels = Y[k:k+minibatch_size].view(-1,1)

        # zero the parameter gradients
        optimizer.zero_grad()

        # forward + backward + optimize
        outputs = net(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()

        # print statistics
        running_loss += loss.item()
        if k % 20 == 19:    # print every 2000 mini-batches
            print('[%d, %5d] loss: %.3f' %
                  (epoch + 1, k + 1, running_loss))
            running_loss = 0.0

print('Finished Training')
pred = buildPred(net, dist_field)
pred = pred.flatten().reshape(res,res)


plt.imshow(pred*np.max(dist_field), cmap=plt.cm.RdYlGn)
plt.show()