import pinocchio as pio
from example_robot_data.robots_loader import *
from example_robot_data import *
import hppfcl
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from quadprog import solve_qp

from solo12_collision_gradient import computeDistJacobian, computeDistJacobianFiniteDiff
from solo_shoulder_approx_torch_nn import *

##### INITIALIZATION METHODS
# Copied from example robot data, remove free flyer base 
def loadSolo(solo=True):
    if solo:
        URDF_FILENAME = "solo.urdf"
    else:
        URDF_FILENAME = "solo12.urdf"
    SRDF_FILENAME = "solo.srdf"
    SRDF_SUBPATH = "/solo_description/srdf/" + SRDF_FILENAME
    URDF_SUBPATH = "/solo_description/robots/" + URDF_FILENAME
    modelPath = getModelPath(URDF_SUBPATH)
    """
    # Load URDF file
    robot = RobotWrapper.BuildFromURDF(modelPath + URDF_SUBPATH) #, [getVisualPath(modelPath)])
                                       #pinocchio.JointModelFreeFlyer())
    # Load SRDF file
    #robot.q0 = readParamsFromSrdf(robot.model, modelPath + SRDF_SUBPATH, False, False, "standing")
    # Add the free-flyer joint limits
    #addFreeFlyerJointLimits(robot.model)
    """
    # SIMPLIFIED
    
    urdf_path = "/home/tnoel/stage/solo-collisions/urdf/"
    mesh_dir = "/opt/openrobots/share/example-robot-data/robots/solo_description"
    #mesh_dir = "/home/tnoel/stage/solo-collisions/urdf/"

    if solo:
        urdf_file = "solo8_simplified.urdf"
    else:
        urdf_file = "solo12_simplified.urdf"
    

    robot = RobotWrapper.BuildFromURDF(urdf_path + urdf_file, [mesh_dir])#, pio.JointModelFreeFlyer())
    robot.q0 = readParamsFromSrdf(robot.model, modelPath + SRDF_SUBPATH, False, False, "standing")
    #robot_config = robot.q0
    
    return robot

# Initialize SOLO model
def initSolo(solo=True):
    robot = loadSolo(solo=solo)
    # Get robot model, data, and collision model
    rmodel = robot.model
    rdata  = rmodel.createData()
    gmodel = robot.collision_model

    # Get the base_link and FL_UPPER_LEG geometries
    base_link_geom = gmodel.getGeometryId("base_link_0")
    FL_upper_leg_geom = gmodel.getGeometryId("FL_UPPER_LEG_0")
    FL_lower_leg_geom = gmodel.getGeometryId("FL_LOWER_LEG_0")
    FR_upper_leg_geom = gmodel.getGeometryId("FR_UPPER_LEG_0")
    FR_lower_leg_geom = gmodel.getGeometryId("FR_LOWER_LEG_0")
    HL_upper_leg_geom = gmodel.getGeometryId("HL_UPPER_LEG_0")
    HL_lower_leg_geom = gmodel.getGeometryId("HL_LOWER_LEG_0")
    HR_upper_leg_geom = gmodel.getGeometryId("HR_UPPER_LEG_0")
    HR_lower_leg_geom = gmodel.getGeometryId("HR_LOWER_LEG_0")

    legs_collGeoms = [FL_upper_leg_geom, 
                 FL_lower_leg_geom,
                 FR_upper_leg_geom, 
                 FR_lower_leg_geom,
                 HL_upper_leg_geom, 
                 HL_lower_leg_geom,
                 HR_upper_leg_geom, 
                 HR_lower_leg_geom]

    # Add the collision pairs to the geometric model
    gmodel.addCollisionPair(pio.CollisionPair(FL_upper_leg_geom, FR_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FL_upper_leg_geom, FR_lower_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FL_lower_leg_geom, FR_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FL_lower_leg_geom, FR_lower_leg_geom))

    #gmodel.addCollisionPair(pio.CollisionPair(FL_upper_leg_geom, HL_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FL_upper_leg_geom, HL_lower_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FL_lower_leg_geom, HL_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FL_lower_leg_geom, HL_lower_leg_geom))
   
    #gmodel.addCollisionPair(pio.CollisionPair(FL_upper_leg_geom, HR_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FL_upper_leg_geom, HR_lower_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FL_lower_leg_geom, HR_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FL_lower_leg_geom, HR_lower_leg_geom))

    #gmodel.addCollisionPair(pio.CollisionPair(FR_upper_leg_geom, HL_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FR_upper_leg_geom, HL_lower_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FR_lower_leg_geom, HL_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FR_lower_leg_geom, HL_lower_leg_geom))

    #gmodel.addCollisionPair(pio.CollisionPair(FR_upper_leg_geom, HR_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FR_upper_leg_geom, HR_lower_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FR_lower_leg_geom, HR_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FR_lower_leg_geom, HR_lower_leg_geom))

    gmodel.addCollisionPair(pio.CollisionPair(HL_upper_leg_geom, HR_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(HL_upper_leg_geom, HR_lower_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(HL_lower_leg_geom, HR_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(HL_lower_leg_geom, HR_lower_leg_geom))

    '''
    for i in range(8):
        for j in range(i+1,8):
            if not (j == i+1):
                gmodel.addCollisionPair(pio.CollisionPair(legs_collGeoms[i], legs_collGeoms[j]))
    '''
    gdata = gmodel.createData()
    return robot, rmodel, rdata, gmodel, gdata

##### NEURAL NETWORK RELATED METHODS
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
    #print(J_pred)
    J_pred = torch.mm(J_pred.view(1,-1),torch.from_numpy(inputJacobian(X_shoulder.view(-1,1), dim)).float())
    print(J_pred)

    print("PyTorch : {}".format(dist_pred + offset))
    return dist_pred, J_pred.numpy()    


##### OTHER METHODS
def getCollisionResults(q, rmodel, rdata, gmodel, gdata):
    pio.computeDistances(rmodel,rdata,gmodel,gdata, q)
    collisions_dist = gdata.distanceResults
    return collisions_dist


def compute_legs_Jdist_avoidance(q, rmodel, rdata, gmodel, gdata):
    counter = 0
    collisions_result = getCollisionResults(q, rmodel, rdata, gmodel, gdata)
    Jdist = []
    dist_vec = []
    pairs = []

    q_des_coll = np.zeros(len(q))
    
    while(counter<len(collisions_result)):
        collDist = collisions_result[counter].min_distance
        p1 = collisions_result[counter].getNearestPoint1()
        p2 = collisions_result[counter].getNearestPoint2()

        frame1 = gmodel.geometryObjects[gmodel.collisionPairs[counter].first].parentFrame
        frame2 = gmodel.geometryObjects[gmodel.collisionPairs[counter].second].parentFrame

        #if collDist < dref:
        #Jlocal_dist = computeDistJacobianFiniteDiff(rmodel, rdata, q, frame1, frame2, p1, p2, floatingBase=False)
        Jlocal_dist = computeDistJacobian(rmodel, rdata, q, frame1, frame2, p1, p2, floatingBase=False)

        dist_vec.append(collDist)
        Jdist.append(Jlocal_dist)
        
        pairs.append(gmodel.collisionPairs[counter])
            
        counter += 1
    return dist_vec, Jdist, pairs


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
        #if collDist < dref:
        Jlocal_dist[q_ind[k]] = np.array(jac)
        Jdist.append(Jlocal_dist)
        dist_vec.append(collDist)

        pairs.append(k)
    return dist_vec, Jdist, pairs


def compute_tau_avoidance(aq, M, b, Jdist, dist_vec, kdist, kv):
    # Solve min_aq .5 aqHaq - gaq s.t. Caq <= d
    d = kdist*dist_vec + kv*Jdist@aq
    C = -Jdist.T
    g = np.zeros(len(aq))
    aq_sol, _, _, _, _, _ = solve_qp(M, g, C, -d)

    aq_sol = aq_sol + aq
    
    tau_q_coll = M@aq_sol + b

    return tau_q_coll
    

def compute_tau_PD(q, q_des, vq, Kp, Kv, q_ind=[]):
    if len(q_ind) == 0 :
        q_ind = [i for i in range(len(q))]
    tau_q = np.zeros(len(q))
    tau_q[q_ind] = -Kp*(q[q_ind] - q_des[q_ind]) - Kv*vq[q_ind]
    return tau_q


def computeAcceleration(M, b, tau):
    return np.linalg.inv(M)@(tau - b) 
