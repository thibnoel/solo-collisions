import numpy as np
import time
from timeit import default_timer as timer
from quadprog import solve_qp
import pinocchio as pio
from example_robot_data.robots_loader import *

import pybullet as pb
import pybullet_data
import pybullet_utils.bullet_client as bc

#from solo12_collision_gradient import computeDistJacobian
#from shoulder_approx.solo_shoulder_approx_torch_nn import *
from solo_collisions_avoidance_control.solo_initialization import *
from solo_collisions_avoidance_control.solo_collisions_viewer import *

from solo_collisions_avoidance_control.solo_coll_wrapper_pytorch import *
from solo_collisions_avoidance_control.solo_coll_wrapper_fcl import *
from solo_collisions_avoidance_control.solo_coll_wrapper_c import *
from solo_collisions_avoidance_control.collisions_controller import *
#from solo_collisions_avoidance_control.solo_c_wrappers import *

def computePDTorque(q, q_des, vq, Kp, Kv, q_ind=[]):
    if len(q_ind) == 0 :
        q_ind = [i for i in range(len(q))]
    tau_q = np.zeros(len(q))
    tau_q[q_ind] = -Kp*(q[q_ind] - q_des[q_ind]) - Kv*vq[q_ind]
    return tau_q


def computeAcceleration(M, b, tau):
    return np.linalg.inv(M)@(tau - b) 


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

### Using directly PyTorch model from python
#trainedModel_path = "/home/tnoel/stage/solo-collisions/src/python/pytorch_data/test_2Dmodel_481.pth"
#trainedModel_path = "/home/tnoel/npy_data/pytorch_data/test_2Dmodel_481.pth"
#trained_model_arch = [[4,8],[8,1]]
#shoulder_model = loadTrainedNeuralNet(trainedModel_path, trained_model_arch)

q_list = []
vq_list = []
tau_q_list = []
dist_list = []
emTrigger_list = []
min_dist = 10*np.ones(20)

# Gen code binding test
#so_file = "/home/tnoel/stage/solo-collisions/compiled_c_lib/libcoll_mod.so"
#nn_so_file = "/home/tnoel/stage/solo-collisions/compiled_c_lib/libcoll_nn.so"
#nn_so_file = "/home/tnoel/stage/solo-collisions/compiled_c_lib/libcoll_nn_shd_knee_large.so"
#cCollFun = CDLL(so_file)
#nnCCollFun = CDLL(nn_so_file)

so_file = "/home/tnoel/stage/solo-collisions/compiled_c_lib/libcoll_legs8_witnessP.so"
#so_file = "/home/tnoel/stage/solo-collisions/compiled_c_lib/libcoll_legs12_witnessP.so"
cCollFun = CDLL(so_file)
avg_exec_time = 0

emergencyFlag = False

# Control parameters
    # Gains
thresh_legs = 0.05
kp_legs = 100
kv_legs = 0.5

thresh_shd = 0.2
kp_shd = 25
kv_shd = 0.1
    # Emergency bounds
q_bounds = [-5,5]
vq_bounds = [-40,40]

### GEPETTO VIEWER
enable_viewer = True

if enable_viewer:
    viewer_coll = initViewer(robot, np.zeros(rmodel.nq), coll=True)


print(physicsClient.getNumJoints(robotID))
for k in range(1200):    
    # Step Bullet simulation
    physicsClient.stepSimulation()
    time.sleep(0.02)

    # Get robot current state
    q, vq = getPosVelJoints(robotID, revoluteJointIndices, physicsClient)

    # Zero torque
    tau_q = np.zeros(robot.model.nq)
    
    # compute dynamic drift -- Coriolis, centrifugal, gravity
    b = pio.rnea(robot.model, robot.data, q, vq, aq0)
    # compute mass matrix M
    M = pio.crba(robot.model, robot.data, q)

    # Initialize dist log
    dist = np.zeros(len(gmodel.collisionPairs))
    #dist = np.zeros(6)
    '''
    ##################### SOLO 12
    nb_motors = 12
    nb_pairs = 20
    wPoints = True
    '''
    '''
    # Get results for the shoulders from a trained neural network
    #shoulders_dist, Jshd, shoulders_pairs = compute_shoulders_Jdist_avoidance(q, shoulder_model, rmodel, rdata, gmodel, gdata, characLength=0.34)
    
    ### Get results from C generated code (shoulder neural net)
    #c_shd_dist, c_shd_jac = getAllShouldersCollisionsResults(q, nnCCollFun, 2, offset=0.08)
    #c_shd_dist, c_shd_jac = getAllShouldersCollisionsResults(q, nnCCollFun, 3, offset=0.11) #offset with 3 inputs: 0.18 (small), 0.11 (large)"

    #tau_q += computeRepulsiveTorque(q, vq, c_shd_dist, c_shd_jac, thresh_shd, kp_shd, kv_shd)

    emTrigger = emergencyCondition(c_dist_legs, q, vq, tau_q, q_bounds, vq_bounds, thresh_legs/10, 50)
    #emergencyFlag = emergencyFlag or emTrigger
    #if(emergencyFlag):
    #    tau_q = computeEmergencyTorque(vq, kv_legs)
    
    emTrigger_list.append(emTrigger)
    
    for i in range(len(legs_pairs)):
        for j in range(len(gmodel.collisionPairs)):
            if(legs_pairs[i] == gmodel.collisionPairs[j]):
                dist[j] = legs_dist[i]
    
    #for i in range(len(c_shd_dist)):
    #    for j in range(4):
    #        dist[len(gmodel.collisionPairs) + j] = c_shd_dist[j]
    
    '''
    ##################### SOLO 8
    nb_motors = 8
    nb_pairs = 6
    wPoints = True
    
    # Get results for the legs from FCL
    legs_dist, Jlegs, legs_pairs, witnessPoints = compute_legs_Jdist_avoidance(q, rmodel, rdata, gmodel, gdata)

    start = timer()
    ### Get results from C generated code
    # Legs
    c_results = getLegsCollisionsResults(q, cCollFun, nb_motors, nb_pairs, witnessPoints=wPoints)
    c_dist_legs = getLegsDistances(c_results, nb_motors, nb_pairs, witnessPoints=wPoints)
    c_Jlegs = getLegsJacobians(c_results, nb_motors, nb_pairs, witnessPoints=wPoints)
    
    c_wPoints = getLegsWitnessPoints(c_results, nb_motors, nb_pairs)

    end = timer()
    avg_exec_time += end-start
    #print("Avg exec time : {:.6f}\r".format(avg_exec_time/(k+1)))

    emergency_d_thresh = thresh_legs/4

    # Compute torque and measure elapsed time
    start = timer()
    #tau_q += computeRepulsiveTorque(q, vq, c_dist_legs, c_Jlegs, thresh_legs, kp_legs, kv_legs)
    tau_q += computeRepulsiveTorque(q, vq, c_dist_legs, c_Jlegs, thresh_legs, kp_legs, kv_legs)
    end = timer()
    #print(end - start)
    
    # Log pairs distances
    for i in range(len(c_dist_legs)):
        dist[i] = c_dist_legs[i]
        if(min_dist[i] > c_dist_legs[i]):
            min_dist[i] = c_dist_legs[i]
    
    
    #tau_q += tau_rep

    # COMPUTE PYBULLET CONTROL
    #tau_q += tau_q_f

    # Save logs
    q_list.append(q)
    vq_list.append(vq)
    tau_q_list.append(tau_q)
    dist_list.append(dist)

    local_wpoints = witnessPoints[5]
    c_local_wpoints = c_wPoints[5]

    #print('FCL')
    #print(local_wpoints)
    #print(Jlegs[3])
    #print('C')
    #print(c_local_wpoints)
    #print(c_Jlegs[3])

    if enable_viewer and k%4==0:
        # GEPETTO VIEWER
        robot.display(q)
        if(len(q)==8):
            caps_frames_list = [["FL_UPPER_LEG", "HL_LOWER_LEG"],\
                                ["FL_LOWER_LEG", "HL_UPPER_LEG"],
                                ["FL_LOWER_LEG", "HL_LOWER_LEG"],

                                ["FR_UPPER_LEG", "HR_LOWER_LEG"],
                                ["FR_LOWER_LEG", "HR_UPPER_LEG"],
                                ["FR_LOWER_LEG", "HR_LOWER_LEG"]]
        
        
        else:
            caps_frames_list = [["FL_UPPER_LEG", "FR_UPPER_LEG"],\
                                ["FL_UPPER_LEG", "FR_LOWER_LEG"],
                                ["FL_LOWER_LEG", "FR_UPPER_LEG"],
                                ["FL_LOWER_LEG", "FR_LOWER_LEG"],
                                
                                ["FL_UPPER_LEG", "HL_LOWER_LEG"],
                                ["FL_LOWER_LEG", "HL_UPPER_LEG"],
                                ["FL_LOWER_LEG", "HL_LOWER_LEG"],

                                ["FL_UPPER_LEG", "HR_LOWER_LEG"],
                                ["FL_LOWER_LEG", "HR_UPPER_LEG"],
                                ["FL_LOWER_LEG", "HR_LOWER_LEG"],
                                
                                ["FR_UPPER_LEG", "HL_LOWER_LEG"],
                                ["FR_LOWER_LEG", "HL_UPPER_LEG"],
                                ["FR_LOWER_LEG", "HL_LOWER_LEG"],
                                
                                ["FR_UPPER_LEG", "HR_LOWER_LEG"],
                                ["FR_LOWER_LEG", "HR_UPPER_LEG"],
                                ["FR_LOWER_LEG", "HR_LOWER_LEG"],
                                
                                ["HL_UPPER_LEG", "HR_UPPER_LEG"],
                                ["HL_UPPER_LEG", "HR_LOWER_LEG"],
                                ["HL_LOWER_LEG", "HR_UPPER_LEG"],
                                ["HL_LOWER_LEG", "HR_LOWER_LEG"]]
        #local_wpoints[1] = [0,0,0]
        #print(local_wpoints)
        for i in range(len(caps_frames_list)):
            color = [0,0,0,0]
            if(c_dist_legs[i]) < 3*thresh_legs:
                color = [0,1,0,1] if c_dist_legs[i] > thresh_legs else [1,0,0,1]
            visualizePair(viewer_coll, rmodel, rdata, q, caps_frames_list[i], c_wPoints[i], color, world_frame=False, init=(k==0))

    # External force 
    
    if k == 99:
        physicsClient.applyExternalForce(robotID,4,[-200,0,0],[0,0,0], pb.LINK_FRAME)
        physicsClient.applyExternalForce(robotID,9,[200,0,0],[0,0,0], pb.LINK_FRAME)
    
    physicsClient.setJointMotorControlArray(robotID,
                                        jointIndices = revoluteJointIndices,
                                        controlMode = pb.TORQUE_CONTROL,
                                        forces = tau_q)


q_list = np.array(q_list)
vq_list = np.array(vq_list)
tau_q_list = np.array(tau_q_list)
dist_list = np.array(dist_list)
emTrigger_list = np.array(emTrigger_list)

print(min_dist)

plt.figure()
#plt.suptitle("kp = {}, kv = {}, threshold = {}".format(kp, kv, thresh))

plt.subplot(1,4,1)
for k in range(q_list.shape[1]):
    plt.plot(q_list[:,k], [i for i in range(len(q_list))],label="q[{}]".format(k))
    #plt.plot(q_list[:,k],label="q[{}]".format(k))
plt.legend()
plt.title("q")

plt.subplot(1,4,2)
for k in range(q_list.shape[1]):
    plt.plot(vq_list[:,k], [i for i in range(len(vq_list))], label="vq[{}]".format(k))
    #plt.plot(tau_q_list[:,k], label="tau_q[{}]".format(k))
plt.legend()
plt.title("v_q")

plt.subplot(1,4,3)
for k in range(q_list.shape[1]):
    plt.plot(tau_q_list[:,k], [i for i in range(len(tau_q_list))], label="tau_q[{}]".format(k))
    #plt.plot(tau_q_list[:,k], label="tau_q[{}]".format(k))
plt.legend()
plt.title("tau_q")

plt.subplot(1,4,4)
for k in range(dist_list.shape[1]):
#for k in range(3,4):
    if(k<20):
        plt.plot(dist_list[:,k], [i for i in range(len(dist_list))], label="d[{}]".format(k), linewidth=1.5)
    else:
        plt.plot(dist_list[:,k], [i for i in range(len(dist_list))], label="d[{}]".format(k) , linewidth=3)
    #plt.plot(dist_list[:,k], label="d[{}]".format(k))

plt.vlines(0,0,len(dist_list)+1, linestyles='solid', colors='black')
plt.vlines(thresh_legs,0,len(dist_list)+1, linestyles='dashed', colors='green')
#plt.vlines(thresh_shd,0,len(dist_list)+1, linestyles='dashed', colors='blue')
plt.legend()
plt.title("Pairs dist")

plt.figure()
plt.plot(emTrigger_list)

plt.show()