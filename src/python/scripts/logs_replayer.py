from solo_collisions_avoidance_control.collisionsViewerClient import *
from solo_collisions_avoidance_control.solo_initialization import *
from solo_collisions_avoidance_control.solo_coll_wrapper_c import *
from solo_collisions_avoidance_control.collisions_controller import *

robot, rmodel, rdata, gmodel, gdata = initSolo(solo=False)
robot1, rmodel1, rdata1, gmodel1, gdata1 = initSolo(solo=False)

nb_motors = 12
nb_pairs = 20

thresh_legs = 0.05
thresh_shd = 0.2

kp_legs = 60
kp_shd = 2.

kv = 0.

#robot.initViewer(loadModel = True)
#robot1.initViewer(loadModel=True, sceneName='ref_scene')

#robot.displayVisuals(False)
#robot.displayCollisions(True)

#gv = robot.viewer.gui
#gv1 = robot1.viewer.gui
#win_id = gv.getWindowID(gv.getWindowList()[0])
viewer_coll = viewerClient(20, 0, thresh_legs, thresh_shd, urdf="/home/tnoel/stage/solo-collisions/urdf/solo12_simplified.urdf", modelPath="/home/tnoel/stage/solo-collisions/urdf")


q_ref = np.load('/home/tnoel/stage/solo-collisions/src/python/q_ref_front_ell_2min.npy')
vq_ref = np.load('/home/tnoel/stage/solo-collisions/src/python/dq_ref_front_ell_2min.npy')

q_log = np.load('/home/tnoel/stage/figures/videos/tnoel_logs/data_2020_10_30_14_58_legs.npz_FILES/q_mes.npy')
vq_log = np.load('/home/tnoel/stage/figures/videos/tnoel_logs/data_2020_10_30_14_58_legs.npz_FILES/v_mes.npy')

'''
q_ref = np.load('/home/tnoel/stage/solo-collisions/src/python/q_ref_shoulder_coll_above_2min.npy')
vq_ref = np.load('/home/tnoel/stage/solo-collisions/src/python/dq_ref_shoulder_coll_above_2min.npy')

q_log = np.load('/home/tnoel/stage/figures/videos/tnoel_logs/data_2020_10_30_15_04_shoulder_alone.npz_FILES/q_mes.npy')
vq_log = np.load('/home/tnoel/stage/figures/videos/tnoel_logs/data_2020_10_30_15_04_shoulder_alone.npz_FILES/v_mes.npy')
'''
so_file = "/home/tnoel/stage/solo-collisions/compiled_c_lib/libcoll_legs12_witnessP.so"
cCollFun = CDLL(so_file)

nn_so_file = "/home/tnoel/stage/solo-collisions/compiled_c_lib/libcoll_nn_shd_knee_large.so"
nnCCollFun = CDLL(nn_so_file)

downsample = 20

#start_time = 94000
#length_time = 1000

start_time = 17000
length_time = 25000

list_tau_legs = np.zeros((length_time, 12))
list_tau_shd = np.zeros((length_time, 12))
list_dist_legs_ref = np.zeros((length_time, nb_pairs))
list_dist_legs_log = np.zeros((length_time, nb_pairs))

list_dist_shd = np.zeros((length_time, 4))

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
FL_FOOT_ID = robot.model.getFrameId('FL_FOOT')
FR_FOOT_ID = robot.model.getFrameId('FR_FOOT')
HL_FOOT_ID = robot.model.getFrameId('HL_FOOT')
HR_FOOT_ID = robot.model.getFrameId('HR_FOOT')

foot1_pos_ref = []
foot1_pos_log = []
foot2_pos_ref = []
foot2_pos_log = []

#for n in gv1.getGroupNodeList('ref_scene'):
    #if not(('FL_U' in n) or ('FL_L' in n) or ('FR_U' in n) or ('FR_L' in n)):
    #    gv1.setVisibility(n,'OFF')
    #gv1.setColor(n, [0.8,0.2,0.2,0.8])
''' 
for n in gv1.getNodeList():
    if 'ref_scene' in n:
        gv1.setColor(n, [0.8,0.2,0.2,0.8])
        #if not(('FL_U' in n) or ('FL_L' in n) or ('FR_U' in n) or ('FR_L' in n) or ('FL_F' in n) or ('FR_F' in n)):
        if not(('HL_U' in n) or ('HL_L' in n) or ('HL_F' in n) or ('HL_H' in n)):
            #gv1.setVisibility(n,'OFF')
            gv1.setColor(n, [0.2,0.8,0.2,0.])
'''

for k in range(length_time):
    q = q_ref[start_time + k][7:]
    vq = vq_ref[start_time + k][6:]

    mes_q = q_log[start_time + k]
    mes_vq = vq_log[start_time + k]

    #pin.forwardKinematics(robot.model,robot.data,q,vq,np.zeros(robot.model.nv))
    #pin.updateFramePlacements(robot.model,robot.data)

    #foot1_pos_ref.append(robot.data.oMf[FL_FOOT_ID].translation.copy())
    #foot2_pos_ref.append(robot.data.oMf[FR_FOOT_ID].translation.copy())

    #pin.forwardKinematics(robot.model,robot.data,mes_q,mes_vq,np.zeros(robot.model.nv))
    #pin.updateFramePlacements(robot.model,robot.data)

    #foot1_pos_log.append(robot.data.oMf[FL_FOOT_ID].translation.copy())
    #foot2_pos_log.append(robot.data.oMf[FR_FOOT_ID].translation.copy())

    ### Get results from C generated code
    # Legs
    c_results_ref = getLegsCollisionsResults(q, cCollFun, nb_motors, nb_pairs, witnessPoints=True)
    c_dist_legs_ref = getLegsDistances(c_results_ref, nb_motors, nb_pairs, witnessPoints=True)
    #c_Jlegs = getLegsJacobians(c_results, nb_motors, nb_pairs, witnessPoints=True)
    
    c_results_log = getLegsCollisionsResults(mes_q, cCollFun, nb_motors, nb_pairs, witnessPoints=True)
    c_dist_legs_log = getLegsDistances(c_results_log, nb_motors, nb_pairs, witnessPoints=True)
    c_Jlegs = getLegsJacobians(c_results_log, nb_motors, nb_pairs, witnessPoints=True)
    c_wPoints = getLegsWitnessPoints(c_results_log, nb_motors, nb_pairs)

    ### Get results from C generated code (shoulder neural net)
    #c_shd_dist, c_shd_jac = getAllShouldersCollisionsResults(q, nnCCollFun, 2, offset=0.08)
    c_shd_dist, c_shd_jac = getAllShouldersCollisionsResults(q, nnCCollFun, 3, offset=0.11)
    
    
    tau_legs = computeRepulsiveTorque(q, vq, c_dist_legs_log, c_Jlegs, thresh_legs, kp_legs, kv, opposeJacIfNegDist=True)
    #tau_shd = computeRepulsiveTorque(q, vq, c_shd_dist, c_shd_jac, thresh_shd, kp_shd, kv, opposeJacIfNegDist=False)
    
    list_dist_legs_ref[k,:] = c_dist_legs_ref
    list_dist_legs_log[k,:] = c_dist_legs_log
    #list_dist_shd[k,:] = c_shd_dist

    list_tau_legs[k,:] = tau_legs
    #list_tau_shd[k,:] = tau_shd
    

    if(k%downsample == 0):
        viewer_coll.display(np.concatenate(([0,0,0,0,0,0,0],mes_q)), c_dist_legs_log, c_shd_dist, c_wPoints, tau_legs, np.zeros(12))
        #robot.display(mes_q)
        #robot1.display(q)
        time.sleep(downsample*1e-3)
        #gv.captureFrame(win_id, 'solo_viewer_images/test_{:05d}.png'.format(k))
'''
foot1_pos_ref = np.array(foot1_pos_ref)
foot1_pos_log = np.array(foot1_pos_log)
foot2_pos_ref = np.array(foot2_pos_ref)
foot2_pos_log = np.array(foot2_pos_log)

act_dof = [6,7,8]
colors = ['r', 'g', 'b', 'y', 'gray', 'purple']

plt.figure()
plt.title("Foot cartesian trajectory with front legs collision")
plt.plot(foot1_pos_ref[:,0], foot1_pos_ref[:,1], c='limegreen', lw=5, label='FL foot ref. traj', linestyle=(0, (1, 10)))
plt.plot(foot2_pos_ref[:,0], foot2_pos_ref[:,1], c='indianred', lw=5, label='FR foot ref. traj', linestyle=(0, (1, 10)))
plt.plot(foot1_pos_log[:,0], foot1_pos_log[:,1], 'g', lw=2, label='FL foot exp. traj')
plt.plot(foot2_pos_log[:,0], foot2_pos_log[:,1], 'r', lw=2, label='FR foot exp. traj')
plt.legend()

plt.figure()
plt.title('q')
for k in act_dof:
    #plt.plot(q_log[:,k+7], label = k)
    plt.plot(q_log[:,k], label = k, c=colors[k-6], lw=2)
    plt.plot(q_ref[:,k+7], label = k, c=colors[k-6], linestyle='dashed')
plt.legend()
plt.figure()
plt.title('dq')
for k in act_dof:
    #plt.plot(vq_log[:,k+6], label = k)
    plt.plot(vq_log[:,k], label = k, c=colors[k-6], lw=2)
    plt.plot(vq_ref[:,k+6], label = k, c=colors[k-6], linestyle='dashed')
plt.legend()
#plt.show()


plt.figure()
pairs = [1,2,3]#[i for i in range(20)]

plt.subplot(2,1,1)
plt.hlines(thresh_legs, 0, length_time, linestyle='dashed')
plt.hlines(0, 0, length_time, color='black', lw=5)
for k in range(len(pairs)):
    #plt.plot(list_dist_legs_ref[:,pairs[k]], label='pair {},{}'.format(caps_frames_list[pairs[k]][0], caps_frames_list[pairs[k]][1]), c=colors[k], linestyle='dashed')
    plt.plot(list_dist_legs_log[:,pairs[k]], c=colors[k], label='pair {},{}'.format(caps_frames_list[pairs[k]][0], caps_frames_list[pairs[k]][1]))
plt.grid(True)
plt.legend()
plt.subplot(2,1,2)
for k in range(12):
    plt.plot(list_tau_legs[:,k], label='tau_q[{}]'.format(k))
plt.grid(True)
plt.legend()
'''