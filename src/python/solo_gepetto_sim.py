import pinocchio as pio
import numpy as np
import matplotlib.pyplot as plt
import time
from example_robot_data.robots_loader import *
from example_robot_data import loadSolo
from solo_control import *


def run_sim(n_steps, q, vq, aq0, q_des):
    active_pairs = []
    active=False

    q_list = []
    tau_q_list = []
    dist_list = []
    pairs = []

    for k in range(n_steps):
        
        # compute dynamic drift -- Coriolis, centrifugal, gravity
        b = pio.rnea(robot.model, robot.data, q, vq, aq0)
        # compute mass matrix M
        M = pio.crba(robot.model, robot.data, q)

        # No torque
        tau_q = np.zeros(robot.model.nv)
        # Compute aq so that M*aq + b = tau_q
        
        zero_aq = np.zeros(len(vq))

        free_aq = computeAcceleration(M, b, tau_q)
        
        k_fixed = 2
        tau_fixed = -k_fixed*M@vq + b
        tau_q += tau_fixed
        
        # Friction torque
        Kf = 0.002
        tau_q += -Kf*vq
        
        #free_aq = computeAcceleration(M, b, tau_q)

        # PD torque
        Kp = 2
        Kv = 0.04
        #q_ind = [0,6,7,8,9,10,11]
        #q_des = robot.q0.copy()
        #q_ind = 0
        #q_des[0] = np.sin(0.01*k)
        #q_des[3] = robot.q0[3] + 0.8
        #tau_q_PD = compute_tau_PD(q, q_des, vq, Kp, Kv)
        #tau_q = compute_tau_PD(q, q_des, vq, Kp, Kv, q_ind=q_ind)   
        #tau_q += tau_q_PD

        # Compute aq so that M*aq + b = tau_q
        #aq = computeAcceleration(M, b, tau_q)
        #tau_q += compute_tau_avoidance(q, M, aq, robot.model, robot.data, gmodel, gdata, dref=0.1)
        dist = 5*np.ones(len(gmodel.collisionPairs) + 4)

        legs_dist, Jlegs, legs_pairs = compute_legs_Jdist_avoidance(q, rmodel, rdata, gmodel, gdata, dref=d_ref_legs)
        shoulders_dist, Jshd, shoulders_pairs = compute_shoulders_Jdist_avoidance(q, shoulder_model, rmodel, rdata, gmodel, gdata, dref=d_ref_shoulders, characLength=0.16)

        #Jdist = []
        #dist_vec = []
        
        #Jshd = []
        
        # Current pb with this approach : optimization is done separately for legs and shoulders
        '''
        if(len(Jlegs) > 0 and len(Jshd) > 0):
            Jdist = np.vstack((Jlegs, Jshd))
            dist_vec = np.concatenate((legs_dist, shoulders_dist))
            
            print(Jdist)
            print(dist_vec)
            tau_coll = compute_tau_avoidance(free_aq, M, b, np.vstack(Jdist), np.array(dist_vec), 100)
            tau_q += k_tau_legs*tau_coll

            for i in range(len(legs_pairs)):
                for j in range(len(gmodel.collisionPairs)):
                    if(legs_pairs[i] == gmodel.collisionPairs[j]):
                        dist[j] = legs_dist[i]
            
            for i in range(len(shoulders_pairs)):
                for j in range(4):
                    if(shoulders_pairs[i] == j):
                        dist[len(gmodel.collisionPairs) + j] = shoulders_dist[i]
        
        #aq = computeAcceleration(M, b, tau_q)
        #free_aq = aq
        '''
        #else:
        
        if len(Jlegs) > 0: 
            tau_legs_coll = compute_tau_avoidance(free_aq, M, b, np.vstack(Jlegs), np.array(legs_dist),100)
            tau_q += k_tau_legs*tau_legs_coll
            
            for i in range(len(legs_pairs)):
                for j in range(len(gmodel.collisionPairs)):
                    if(legs_pairs[i] == gmodel.collisionPairs[j]):
                        dist[j] = legs_dist[i]
            

        '''
        if len(Jshd) > 0: 
            tau_shoulders_coll = compute_tau_avoidance(free_aq, M, b, np.vstack(Jshd), np.array(shoulders_dist),100)
            tau_q += k_tau_shoulders*tau_shoulders_coll

            for i in range(len(shoulders_pairs)):
                for j in range(4):
                    if(shoulders_pairs[i] == j):
                        dist[len(gmodel.collisionPairs) + j] = shoulders_dist[i]
        '''   
        #tau_q += 5*tau_q_legs_coll
        aq = computeAcceleration(M, b, tau_q)

        vq += aq*dt
        q = pio.integrate(robot.model, q, vq*dt)

        robot.display(q) 
        time.sleep(0.001)

        q_list.append(q)
        tau_q_list.append(tau_q)
        dist_list.append(dist)
    return q_list, tau_q_list, dist_list



trainedModel_path = "/home/tnoel/stage/solo-collisions/src/python/pytorch_data/test_2Dmodel_481.pth"
shoulder_model = loadTrainedNeuralNet(trainedModel_path)

robot, rmodel, rdata, gmodel, gdata = initSolo()
robot_config = robot.q0




enableGUI = True

if(enableGUI):
    robot.initViewer(loadModel=True)
    gv = robot.viewer.gui
    # Display the robot
    robot.rebuildData() 
    robot.displayCollisions(False)
    robot.displayVisuals(True)
    robot.display(robot_config) 
    '''
    for n in gv.getNodeList():
        if 'collision' in n and len(n)>27:
            gv.setVisibility(n,'OFF')
            gv.setColor(n, [1,1,1,1])
    '''
    gv.refresh()

dt = 1e-3

#q = -np.pi + 2*np.pi*np.random.rand(robot.model.nq)
q = robot.q0.copy()
q[0] -= 0.6
q[3] += .7
q[6] -= 1.1
#q[9] += 0.3*(-np.pi + 2*np.pi*np.random.rand())
#q[5] -= 1
#q[6] += 1
#q[9] -= 1
#q+= -0.02 + 0.04*np.random.rand(12)
#vq = np.random.rand(robot.model.nv)
vq = np.zeros(robot.model.nv)
vq[0] = 1
aq0 = np.zeros(robot.model.nv)

q_des = robot.q0.copy()
#q_des[3] = 0.5
#q_des[4] -= 0.2
#q_des[5] -= 1
robot.display(q) 





d_ref_legs = 0.11
d_ref_shoulders = 0.2

#d_ref_legs = 0.06
#d_ref_shoulders = 0.25

k_tau_legs = 1
k_tau_shoulders = 1




q_list, tau_q_list, dist_list = run_sim(1000, q, vq, aq0, q_des)

q_list = np.array(q_list)
tau_q_list = np.array(tau_q_list)
dist_list = np.array(dist_list)

plt.figure()

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
plt.vlines(d_ref_legs,0,len(dist_list)+1, linestyles='dashed', colors='green')
plt.vlines(d_ref_shoulders,0,len(dist_list)+1, linestyles='dashed', colors='green')
plt.legend()
plt.title("Pairs dist")

plt.show()
