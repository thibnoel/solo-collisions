import numpy as np
import time
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

    robotID = physicsClient.loadURDF("solo12_simplified_bullet.urdf", robotStartPos, robotStartOrientation, useFixedBase=1)
    # Initialize velocity control 
    revoluteJointIndices = [0,1,2,4,5,6,8,9,10,12,13,14]
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
robotID, revoluteJointIndices, physicsClient = loadBulletPhysicsClient(dt)

robot, rmodel, rdata, gmodel, gdata = initSolo()
aq0 = np.zeros(robot.model.nv)

trainedModel_path = "/home/tnoel/stage/solo-collisions/src/python/pytorch_data/test_2Dmodel_481.pth"
shoulder_model = loadTrainedNeuralNet(trainedModel_path)

q_list = []
tau_q_list = []
dist_list = []


# Gen code binding test
so_file = "/home/tnoel/stage/solo-collisions/libcoll_mod.so"
cCollFun = CDLL(so_file)


print(physicsClient.getNumJoints(robotID))
for k in range(400):    
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
    tau_q = np.zeros(12)
    # PD torque    
    q_des = robot.q0.copy()
    q_des[0] += 2*np.sin(0.01*k)
    q_des[3] += 0.5
    Kp = 20
    Kv = 0.05
    tau_q_PD = compute_tau_PD(q, q_des, vq, Kp, Kv)#, q_ind=q_ind)
    # Friction torque
    Kf = 0.002
    tau_q_f = -Kf * vq
    # Zero acceleration torque
    k_fixed = 25
    tau_fixed = -k_fixed*M@vq + b
    
    # Initialize dist log
    dist = 5*np.zeros(len(gmodel.collisionPairs) + 4)
    
    # Compute reference acceleration
    ref_aq = computeAcceleration(M, b, tau_q_f)

    # Get results for the legs from FCL
    legs_dist, Jlegs, legs_pairs = compute_legs_Jdist_avoidance(q, rmodel, rdata, gmodel, gdata)
    # Get results for the shoulders from a trained neural network
    shoulders_dist, Jshd, shoulders_pairs = compute_shoulders_Jdist_avoidance(q, shoulder_model, rmodel, rdata, gmodel, gdata, characLength=0.34)

    ### Get results from C generated code
    c_results = getLegsCollisionsResults(q, cCollFun)
    c_dist_legs = getDistances(c_results)
    c_Jlegs = getJacobians(c_results)

    legs_dist = c_dist_legs
    Jlegs = c_Jlegs

    # Safety distance 
    safety_dist = 0.1
    legs_dist = legs_dist - safety_dist

    kdist = 1000
    kv = 50
    
    Jdist = np.vstack((Jlegs, Jshd))
    dist_vec = np.concatenate((legs_dist, shoulders_dist))
    
    # Compute avoidance torque from ref. acceleration
    tau_coll = compute_tau_avoidance(ref_aq, M, b, -np.vstack(Jdist), np.array(dist_vec), kdist,kv)
    tau_q += tau_coll

    for i in range(len(legs_pairs)):
        for j in range(len(gmodel.collisionPairs)):
            if(legs_pairs[i] == gmodel.collisionPairs[j]):
                dist[j] = legs_dist[i] + safety_dist
    
    for i in range(len(shoulders_pairs)):
        for j in range(4):
            if(shoulders_pairs[i] == j):
                dist[len(gmodel.collisionPairs) + j] = shoulders_dist[i]

    # Save logs
    q_list.append(q)
    tau_q_list.append(tau_q)
    dist_list.append(dist)

    physicsClient.setJointMotorControlArray(robotID,
                                        jointIndices = revoluteJointIndices,
                                        controlMode = pb.TORQUE_CONTROL,
                                        forces = tau_q)

    #print(q)


q_list = np.array(q_list)
tau_q_list = np.array(tau_q_list)
dist_list = np.array(dist_list)

plt.figure()
plt.suptitle("k_d = {}, k_v = {}, safe_dist = {}".format(kdist, kv, safety_dist))

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
    plt.plot(dist_list[:,k], [i for i in range(len(dist_list))], label="d[{}]".format(k))
    #plt.plot(dist_list[:,k], label="d[{}]".format(k))
plt.vlines(0,0,len(dist_list)+1, linestyles='solid', colors='black')
#plt.vlines(d_ref_legs,0,len(dist_list)+1, linestyles='dashed', colors='green')
#plt.vlines(d_ref_shoulders,0,len(dist_list)+1, linestyles='dashed', colors='green')
plt.legend()
plt.title("Pairs dist")

plt.show()