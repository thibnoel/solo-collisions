from collision_approx.collision_approx_pytorch import *


def loadTrainedNeuralNet(trainedModel_path):
    # Load trained model
    trainedNet = Net([[4,8],[8,1]], activation=torch.tanh)
    trainedNet.load_state_dict(torch.load(trainedModel_path))
    # Set model to eval mode
    trainedNet.eval()
    return trainedNet


def qToTorchInput(q):
    dim = len(q)
    X = np.zeros(2*dim)
    for k in range(dim):
        X[k] = np.cos(q[k])
        X[dim+k] = np.sin(q[k])
    X = torch.from_numpy(X).float()
    return X


def neuralNetShoulderResult(trainedNet, q_shoulder, offset):
    dim = len(q_shoulder)
    X_shoulder = qToTorchInput(q_shoulder)
    #dist = trainedNet(X_shoulder).data
    dist_pred = trainedNet(X_shoulder).item() - offset

    J_pred = trainedNet.jacobian(X_shoulder)
    J_pred = torch.mm(J_pred.view(1,-1),torch.from_numpy(inputJacobian(X_shoulder.view(-1,1), dim)).float())

    return dist_pred, J_pred.numpy() 


def compute_shoulders_Jdist_avoidance(q, shoulder_model, rmodel, rdata, gmodel, gdata, characLength=0.16):
    Jdist = []
    dist_vec = []
    pairs = []
    
    FL_ind = [0,1]
    FR_ind = [3,4]
    HL_ind = [6,7]
    HR_ind = [9,10]
    q_FL_shoulder = q[FL_ind].copy()
    q_FR_shoulder = q[FR_ind].copy()
    q_HL_shoulder = q[HL_ind].copy()
    q_HR_shoulder = q[HR_ind].copy()

    def evalModel(shoulder_ind, shoulder_q, pred_offset, shoulder_sym=[1,1]):
        shoulder_q[0] = shoulder_sym[0]*shoulder_q[0]
        shoulder_q[1] = shoulder_sym[1]*shoulder_q[1]

        collDist, jac = neuralNetShoulderResult(shoulder_model, shoulder_q, pred_offset)
        collDist = characLength*collDist
        jac = jac[0]
        jac[0] = shoulder_sym[0]*jac[0]
        jac[1] = shoulder_sym[1]*jac[1]

        return collDist, jac

    shoulder_syms = [[1,1], [-1,1], [1,-1], [-1,-1]]
    q_ind = [FL_ind, FR_ind, HL_ind, HR_ind]
    shoulders_q = [q_FL_shoulder, q_FR_shoulder, q_HL_shoulder, q_HR_shoulder]
    
    for k in range(4):
        Jlocal_dist = np.zeros(len(q))
        collDist, jac = evalModel(q_ind[k], shoulders_q[k], 0.08, shoulder_sym=shoulder_syms[k])

        Jlocal_dist[q_ind[k]] = np.array(jac)
        Jdist.append(Jlocal_dist)
        dist_vec.append(collDist)

        pairs.append(k)
    return dist_vec, Jdist, pairs