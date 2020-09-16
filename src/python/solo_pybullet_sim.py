import numpy as np
import time
from timeit import default_timer as timer
from quadprog import solve_qp
import pinocchio as pio
from example_robot_data.robots_loader import *

import pybullet as pb
import pybullet_data
import pybullet_utils.bullet_client as bc

from solo12_collision_gradient import computeDistJacobian
from solo_shoulder_approx_torch_nn import *
from solo_control import *
from legs_cpp_wrapper import *
from shoulders_cpp_wrapper import *


def getPosVelJoints(robotID, revoluteJointIndices, physicsClient):
    jointStates = physicsClient.getJointStates(robotID, revoluteJointIndices)
    #baseState = physicsClient.getBasePositionAndOrientation(robotID) UNUSED FREE FLYING BASE

    q = np.array([jointStates[i_joint][0] for i_joint in range(len(jointStates))])
    qdot = np.array([jointStates[i_joint][1] for i_joint in range(len(jointStates))])

    return q, qdot

def loadBulletPhysicsClient(dt, enableGUI=True, grav=-9.81):
    # Start the physics client
    if enableGUI:
        physicsClient = bc.BulletClient(connection_mode=pb.GUI)
    else:
        physicsClient = bc.BulletClient(connection_mode=pb.DIRECT)

    physicsClient.setGravity(0, 0, grav)
    physicsClient.setTimeStep(dt)
    # Load an horizontal plane
    physicsClient.setAdditionalSearchPath(pybullet_data.getDataPath())
    groundPlaneId = physicsClient.loadURDF("plane.urdf")
    # Load the robot
    robotStartPos = [0, 0, 0.4]
    robotStartOrientation = pb.getQuaternionFromEuler([0, 0, 0])
    physicsClient.setAdditionalSearchPath("/opt/openrobots/share/example-robot-data/robots/solo_description")
    physicsClient.setAdditionalSearchPath("/home/tnoel/stage/solo-collisions/urdf")

    #robotID = physicsClient.loadURDF("solo12_simplified_bullet.urdf", robotStartPos, robotStartOrientation, useFixedBase=1)
    robotID = physicsClient.loadURDF("solo8_simplified_bullet.urdf", robotStartPos, robotStartOrientation, useFixedBase=1)
    # Initialize velocity control 
    #revoluteJointIndices = [0,1,2,4,5,6,8,9,10,12,13,14]
    revoluteJointIndices = [0,1,3,4,6,7,9,10]
    
    physicsClient.setJointMotorControlArray(robotID,
                                            jointIndices = revoluteJointIndices,
                                            controlMode = pb.VELOCITY_CONTROL,
                                            targetVelocities = [0.0 for m in revoluteJointIndices],
                                            forces = [0.0 for m in revoluteJointIndices])
    # Switch to torque control
    physicsClient.setJointMotorControlArray(robotID,
                                            jointIndices = revoluteJointIndices,
                                            controlMode = pb.TORQUE_CONTROL,
                                            forces = [0.0 for m in revoluteJointIndices])                       
    # Initialize physics with 1 step
    physicsClient.stepSimulation()

    return robotID, revoluteJointIndices, physicsClient

enableGUI = True
dt = 1e-3
grav = -9.81
robotID, revoluteJointIndices, physicsClient = loadBulletPhysicsClient(dt, enableGUI=False)

robot, rmodel, rdata, gmodel, gdata = initSolo(solo=True)
aq0 = np.zeros(robot.model.nv)

trainedModel_path = "/home/tnoel/stage/solo-collisions/src/python/pytorch_data/test_2Dmodel_481.pth"
shoulder_model = loadTrainedNeuralNet(trainedModel_path)

q_list = []
tau_q_list = []
dist_list = []


# Gen code binding test
#so_file = "/home/tnoel/stage/solo-collisions/libcoll_mod.so"
#nn_so_file = "/home/tnoel/stage/solo-collisions/libcoll_nn.so"
#cCollFun = CDLL(so_file)
#nnCCollFun = CDLL(nn_so_file)

so_file = "/home/tnoel/stage/solo-collisions/libcoll_legs8.so"
cCollFun = CDLL(so_file)

print(physicsClient.getNumJoints(robotID))
for k in range(300):    
    # Step Bullet simulation
    physicsClient.stepSimulation()
    time.sleep(0.02)

    # Get robot current state
    q, vq = getPosVelJoints(robotID, revoluteJointIndices, physicsClient)
    
    # compute dynamic drift -- Coriolis, centrifugal, gravity
    b = pio.rnea(robot.model, robot.data, q, vq, aq0)
    # compute mass matrix M
    M = pio.crba(robot.model, robot.data, q)
    
    # Zero torque
    tau_q = np.zeros(robot.model.nq)
    # PD torque    
    q_des = robot.q0.copy()
    q_des[0] += 2*np.sin(0.01*k)
    q_des[3] += 0.5
    Kp = 20
    Kv = 0.4
    tau_q_PD = compute_tau_PD(q, q_des, vq, Kp, Kv)#, q_ind=q_ind)
    # Friction torque
    Kf = 0.002
    tau_q_f = -Kf * vq
    # Zero acceleration torque
    k_fixed = 100
    tau_fixed = -k_fixed*M@vq + b
    
    # Initialize dist log
    #dist = 5*np.zeros(len(gmodel.collisionPairs) + 4)
    dist = np.zeros(6)
    '''
    # Compute reference acceleration
    ref_aq = computeAcceleration(M, b, tau_q_f)

    ##################### SOLO 12
    # Get results for the legs from FCL
    legs_dist, Jlegs, legs_pairs = compute_legs_Jdist_avoidance(q, rmodel, rdata, gmodel, gdata)
    # Get results for the shoulders from a trained neural network
    shoulders_dist, Jshd, shoulders_pairs = compute_shoulders_Jdist_avoidance(q, shoulder_model, rmodel, rdata, gmodel, gdata, characLength=0.34)

    ### Get results from C generated code
    c_results = getLegsCollisionsResults(q, cCollFun)
    c_dist_legs = getDistances(c_results)
    c_Jlegs = getJacobians(c_results)

    c_shoulder_FL = getShoulderCollisionsResults(q[0:2], nnCCollFun)

    c_FLsh_dist = getDistance(c_shoulder_FL)
    c_JFLsh = getJacobian(c_shoulder_FL)
    print("C : {}".format(c_FLsh_dist))
    print(c_JFLsh)

    legs_dist = c_dist_legs
    Jlegs = c_Jlegs

    # Safety distance 
    #safety_dist = 0.1
    #legs_dist = legs_dist - safety_dist

    #kdist = 1000
    #kv = 50

    #tau_q += tau_q_PD
    
    Jdist = np.vstack((Jlegs, Jshd))
    dist_vec = np.concatenate((legs_dist, shoulders_dist))
    
    thresh = 0.05
    kp = 25
    kv = 0.08
    #for i in range(len(dist_vec)):
    for i in range(len(dist_vec)):
        J = Jdist[i]
        d = dist_vec[i]

        tau_rep = np.zeros(12)
        if(d < thresh):
            tau_rep = -kp*(d - thresh) - kv*J@vq
            tau_rep *= np.exp(thresh/(5*k*(d-thresh)))
        #if(d < thresh/5):
        #    tau_rep = -5*kp*(d - thresh) - 5*kv*J@vq
        tau_q += tau_rep*J.T

    
    for i in range(len(legs_pairs)):
        for j in range(len(gmodel.collisionPairs)):
            if(legs_pairs[i] == gmodel.collisionPairs[j]):
                dist[j] = legs_dist[i]
    
    for i in range(len(shoulders_pairs)):
        for j in range(4):
            if(shoulders_pairs[i] == j):
                dist[len(gmodel.collisionPairs) + j] = shoulders_dist[i]
    '''
    ##################### SOLO 8
    ### Get results from C generated code
    start = timer()
    c_results = getLegsCollisionsResults8(q, cCollFun)
    c_dist_legs = getDistances8(c_results)
    c_Jlegs = getJacobians8(c_results)
    

    #print(c_dist_legs)
    legs_dist = c_dist_legs
    Jdist = c_Jlegs


    thresh = 0.12
    kp = 50
    kv = 0.5
    #for i in range(len(dist_vec)):
    for i in range(len(legs_dist)):
        J = Jdist[i]
        d = legs_dist[i]

        tau_rep = np.zeros(8)
        if(d < thresh):
            tau_rep = -kp*(d - thresh)**2/(thresh*thresh) - kv*J@vq
            #tau_rep *= np.exp(thresh/(5*k*(d-thresh)))
        #if(d < thresh/5):
        #    tau_rep = -5*kp*(d - thresh) - 5*kv*J@vq
        tau_q += tau_rep*J.T

    for i in range(len(legs_dist)):
        #for j in range(len(gmodel.collisionPairs)):
        #    if(legs_pairs[i] == gmodel.collisionPairs[j]):
        dist[i] = legs_dist[i]

    end = timer()
    print(end - start)

    # COMPUTE PYBULLET CONTROL
    #tau_q += tau_q_f

    # Save logs
    q_list.append(q)
    tau_q_list.append(tau_q)
    dist_list.append(dist)

    # External force 
    if k == 99:
        physicsClient.applyExternalForce(robotID,4,[-500,0,0],[0,0,0], pb.LINK_FRAME)
        physicsClient.applyExternalForce(robotID,10,[500,0,0],[0,0,0], pb.LINK_FRAME)

    physicsClient.setJointMotorControlArray(robotID,
                                        jointIndices = revoluteJointIndices,
                                        controlMode = pb.TORQUE_CONTROL,
                                        forces = tau_q)

    #print(q)


q_list = np.array(q_list)
tau_q_list = np.array(tau_q_list)
dist_list = np.array(dist_list)

plt.figure()
plt.suptitle("kp = {}, kv = {}, threshold = {}".format(kp, kv, thresh))

plt.subplot(1,3,1)
for k in range(q_list.shape[1]):
    plt.plot(q_list[:,k], [i for i in range(len(q_list))],label="q[{}]".format(k))
    #plt.plot(q_list[:,k],label="q[{}]".format(k))
plt.legend()
plt.title("q")

plt.subplot(1,3,2)
for k in range(q_list.shape[1]):
    plt.plot(tau_q_list[:,k], [i for i in range(len(tau_q_list))], label="tau_q[{}]".format(k))
    #plt.plot(tau_q_list[:,k], label="tau_q[{}]".format(k))
plt.legend()
plt.title("tau_q")

plt.subplot(1,3,3)
for k in range(dist_list.shape[1]):
#for k in range(3,4):
    if(k<20):
        plt.plot(dist_list[:,k], [i for i in range(len(dist_list))], label="d[{}]".format(k), linewidth=1.5)
    else:
        plt.plot(dist_list[:,k], [i for i in range(len(dist_list))], label="d[{}]".format(k) , linewidth=3)
    #plt.plot(dist_list[:,k], label="d[{}]".format(k))

plt.vlines(0,0,len(dist_list)+1, linestyles='solid', colors='black')
plt.vlines(thresh,0,len(dist_list)+1, linestyles='dashed', colors='green')
#plt.vlines(d_ref_shoulders,0,len(dist_list)+1, linestyles='dashed', colors='green')
plt.legend()
plt.title("Pairs dist")

plt.show()