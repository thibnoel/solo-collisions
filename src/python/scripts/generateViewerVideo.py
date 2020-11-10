from solo_collisions_avoidance_control.solo_initialization import *
from solo_collisions_avoidance_control.solo_coll_wrapper_c import *
from solo_collisions_avoidance_control.collisionsViewerClient import *

def addSceneCylinder(gv, scene, name, placement, color):
    gv.addCylinder(scene + '/pinocchio/collisions/cylinder_' + name, .01, .003, color)
    gv.applyConfiguration(scene + '/pinocchio/collisions/cylinder_' + name,pin.SE3ToXYZQUATtuple(placement))
    gv.setColor(scene + '/pinocchio/collisions/cylinder_' + name, color)

def addSceneLine(gv, scene, name, endPoints, color, translation=np.array([0,0,0])):
    gv.addLine(scene + '/pinocchio/collisions/line_' + name, endPoints[0].tolist(), endPoints[1].tolist(), color)
    gv.setLineExtremalPoints(scene + '/pinocchio/collisions/line_' + name, (endPoints[0]+translation).tolist(), (endPoints[1]+translation).tolist())
    gv.setColor(scene + '/pinocchio/collisions/line_' + name, color)

def addSceneWitnessLine(gv, scene, name, endPoints, color, translation=np.array([0,0,0])):
    p1 = endPoints[0]+translation
    p2 = endPoints[1]+translation

    #direc = (p2-p1)/np.linalg.norm(p2-p1) 
    M1 = pin.SE3(pin.Quaternion.FromTwoVectors(np.matrix([0,0,1]).T,p1-p2).matrix(),p1)
    M2 = pin.SE3(pin.Quaternion.FromTwoVectors(np.matrix([0,0,1]).T,p2-p1).matrix(),p2)
    
    addSceneLine(gv, scene, name, endPoints, color, translation=translation)
    addSceneCylinder(gv, scene, name + '_0', M1, color)
    addSceneCylinder(gv, scene, name + '_1', M2, color)

def addScenePairWitness(gv, scene, rmodel, rdata, q, framesPair, local_wpoints, color, translation=np.array([0,0,0])):
    p0 = np.array(local_wpoints[0])
    p1 = np.array(local_wpoints[1])

    frame0 = rmodel.getFrameId(framesPair[0])   
    frame1 = rmodel.getFrameId(framesPair[1])

    p0 = rdata.oMf[frame0].rotation@p0 + rdata.oMf[frame0].translation
    p1 = rdata.oMf[frame1].rotation@p1 + rdata.oMf[frame1].translation

    addSceneWitnessLine(gv, scene, framesPair[0] + framesPair[1], [p0,p1], color, translation=translation)

def addSceneMultiplePairsWitness(gv, scene, rmodel, rdata, q, framesPairList, wpointsList, distList, translation=np.array([0,0,0])):
    inactive_color = [0,0,0,0]
    viewable_color = [0,1,0,1]
    active_color = [1,0,0,1]

    pin.forwardKinematics(rmodel, rdata, q)
    pin.updateFramePlacements(rmodel, rdata)

    for k in range(len(framesPairList)):
        color = inactive_color
        if(distList[k] < 0.15):
            color = viewable_color
        if(distList[k] < 0.05):
            color = active_color
        addScenePairWitness(gv, scene, rmodel, rdata, q, framesPairList[k], wpointsList[k], color, translation=translation)

def plotPairsDist(distVecList, pairs_names):
    local_plots = []
    for k in range(len(distVecList[0])):
        local_plots.append(plt.plot(distVecList[:,k], label='Pair ' + pairs_names[k][0][:6] + '_' + pairs_names[k][1][:6]))
    plt.legend()
    return local_plots

def visualizeShoulderBackground(q, shd_dist_landscape, activation_thresh, dim=2):
    shd_dist_landscape = 1*(shd_dist_landscape > 0) + 1*(shd_dist_landscape > activation_thresh) 
    shoulders_names = ['FL', 'FR', 'HL', 'HR']
    shoulders_syms = [[1,1],[-1,1], [1,-1], [-1,-1]]
    for k in range(4):
        if dim==2:
            local_dist_landscape = shd_dist_landscape.copy()
        elif dim==3:
            #ind = 0
            ind = int(len(shd_dist_landscape)*((q[7 + k*3 + 2]%(2*np.pi))/(2*np.pi)))
            local_dist_landscape = shd_dist_landscape[ind].copy()

        if(shoulders_syms[k][0] == -1):
            local_dist_landscape = np.flip(local_dist_landscape, axis = 1)
        if(shoulders_syms[k][1] == -1):
            local_dist_landscape = np.flip(local_dist_landscape, axis = 0)
        
        plt.subplot(2,2,k+1)
        #plt.imshow(shd_dist_landscape, extent=[-np.pi, np.pi, -np.pi, np.pi], cmap=plt.cm.gray)
        plt.imshow(local_dist_landscape, extent=[-np.pi, np.pi, -np.pi, np.pi], cmap=plt.cm.afmhot)

def visualizeShoulderDist(q_shoulder, dist, shd_thresh):
    plt.axis([-np.pi, np.pi, -np.pi, np.pi])
    if dist < shd_thresh:
        color = 'r'
    else:
        color = 'limegreen'
    out = plt.scatter(q_shoulder[0], q_shoulder[1], c=color, alpha=1)
    
    return out

def visualizeShouldersCollisions(qplots, q, shd_dist, activation_thresh, dim=2):

    shoulders_names = ['FL', 'FR', 'HL', 'HR']
    shoulders_syms = [[1,1],[-1,1], [1,-1], [-1,-1]]
    for k in range(4):
        plt.subplot(2,2,k+1)
        plt.title(shoulders_names[k] + '\nd = {:.3f}'.format(shd_dist[k]))
        qplots[k].append(visualizeShoulderDist(q[7+3*k:7+3*k+2].tolist(), shd_dist[k], activation_thresh))

        if (len(qplots[k]) > 4):
            qplots[k].pop(0).remove()


def lerp_value(val_range, duration, curr_time, start_time):
    if curr_time <= start_time:
        return val_range[0]
    if curr_time > start_time and curr_time < start_time + duration :
        return val_range[0] + ((curr_time - start_time)/duration)*(val_range[1] - val_range[0])
    else:
        return val_range[1]

def lerp_vector(ranges, duration, curr_time, start_time):
    n = len(ranges[0])
    new_vec = np.zeros(n)
    for k in range(n):
        new_vec[k] = lerp_value([ranges[0][k], ranges[1][k]], duration, curr_time, start_time)
    return new_vec

def lerp_transform(targets, duration, curr_time, start_time):
    t0 = targets[0]
    t1 = targets[1]
    M0 = pin.SE3(pin.Quaternion(np.matrix(t0[3:]).T),np.array(t0[0:3]))
    M1 = pin.SE3(pin.Quaternion(np.matrix(t1[3:]).T),np.array(t1[0:3]))

    if curr_time <= start_time:
        return t0
    if curr_time > start_time and curr_time < start_time + duration :
        M = pin.SE3.Interpolate(M0, M1, (curr_time - start_time)/duration)
        return pin.SE3ToXYZQUATtuple(M)
    else :
        return t1


def getSceneNodes(gv, sceneName):
    nodes = []
    for n in gv.getNodeList():
        if sceneName in n:
            nodes.append(n)
    return nodes

def checkLinkList(node, linkList):
    for i in range(len(linkList)):
        if linkList[i] in node:
            return True
    return False

def getLegsCollResults(q):
    nb_motors = 12
    nb_pairs = 20
    ### Get results from C generated code
    # Legs
    c_results = getLegsCollisionsResults(q, cCollFun, nb_motors, nb_pairs, witnessPoints=True)
    c_dist_legs = getLegsDistances(c_results, nb_motors, nb_pairs, witnessPoints=True)
    c_Jlegs = getLegsJacobians(c_results, nb_motors, nb_pairs, witnessPoints=True)
    c_wPoints = getLegsWitnessPoints(c_results, nb_motors, nb_pairs)

    return c_dist_legs.copy(), c_Jlegs.copy(), c_wPoints.copy()

# C lib.
so_file = "/home/tnoel/stage/solo-collisions/compiled_c_lib/libcoll_legs12_witnessP.so"
cCollFun = CDLL(so_file)
nn_so_file = "/home/tnoel/stage/solo-collisions/compiled_c_lib/libcoll_nn_shd_knee_large.so"
nnCCollFun = CDLL(nn_so_file)

##### vid 27 : on stand
#q_log = np.load('/home/tnoel/stage/figures/videos/tnoel_logs/data_low_gains_1407.npz_FILES/q_mes.npy')
#ref_cam_transform = [0.8625903725624084, -1.22608482837677, 0.5948385000228882, 0.5008015632629395, 0.15756861865520477, 0.24863572418689728, 0.8139718770980835]

##### vid 28 : on stand
#q_log = np.load('/home/tnoel/stage/figures/videos/tnoel_logs/data_1414.npz_FILES/q_mes.npy')
#ref_cam_transform = [0.8625903725624084, -1.22608482837677, 0.5948385000228882, 0.5008015632629395, 0.15756861865520477, 0.24863572418689728, 0.8139718770980835]

##### vid 29 : on stand
#q_log = np.load('/home/tnoel/stage/figures/videos/tnoel_logs/data_1418.npz_FILES/q_mes.npy')
#ref_cam_transform = [0.8625903725624084, -1.22608482837677, 0.5948385000228882, 0.5008015632629395, 0.15756861865520477, 0.24863572418689728, 0.8139718770980835]

##### vid 50 : legs (ref traj)
#q_ref = np.load('/home/tnoel/stage/solo-collisions/src/python/q_ref_front_ell_2min.npy')
#q_log = np.load('/home/tnoel/stage/figures/videos/tnoel_logs/data_2020_10_30_14_58_legs.npz_FILES/q_mes.npy')
#ref_cam_transform = [0.9872623085975647, 0.5959295630455017, 0.12226107716560364, 0.2987486720085144, 0.5386072397232056, 0.6919279098510742, 0.3766791522502899]

##### vid 53 : shoulder (rej traj)
#q_ref = np.load('/home/tnoel/stage/solo-collisions/src/python/q_ref_shoulder_coll_above_2min.npy')
#q_log = np.load('/home/tnoel/stage/figures/videos/tnoel_logs/data_2020_10_30_15_04_shoulder_alone.npz_FILES/q_mes.npy')
#ref_cam_transform_0 = [-0.8647813200950623, 0.7156628966331482, 0.6919267177581787, -0.19916008412837982, 0.4242538511753082, 0.8002442717552185, -0.374102920293808]
#ref_cam_transform_1 = [-0.41886797547340393, -0.6189035177230835, 0.6099254488945007, 0.35977357625961304, -0.12233929336071014, -0.2324928343296051, 0.895289421081543]
#ref_cam_transform = ref_cam_transform_0
#[-0.8647813200950623, 0.7156628966331482, 0.6919267177581787, -0.19916008412837982, 0.4242538511753082, 0.8002442717552185, -0.374102920293808]

##### vid 56 : shoulder (ref traj) + caps
q_ref = np.load('/home/tnoel/stage/solo-collisions/src/python/q_ref_shoulder_coll_above_2min.npy')
q_log = np.load('/home/tnoel/stage/figures/videos/tnoel_logs/data_2020_10_30_15_06_shoulder_caps.npz_FILES/q_mes.npy')
ref_cam_transform = [-0.5344892740249634, -0.9739723205566406, 0.6105204224586487, 0.4669668972492218, -0.11174317449331284, -0.22299057245254517, 0.8483693599700928]

##### vid 55 : no log



# Shoulder alone
#q_ref = np.load('/home/tnoel/stage/solo-collisions/src/python/q_ref_shoulder_coll_above_2min.npy')
#q_log = np.load('/home/tnoel/stage/figures/videos/tnoel_logs/data_2020_10_30_15_04_shoulder_alone.npz_FILES/q_mes.npy')

# Front legs
#q_ref = np.load('/home/tnoel/stage/solo-collisions/src/python/q_ref_front_ell_2min.npy')
#q_log = np.load('/home/tnoel/stage/figures/videos/tnoel_logs/data_2020_10_30_14_58_legs.npz_FILES/q_mes.npy')


robot0, rmodel0, rdata0, gmodel0, gdata0 = initSolo(solo=False)
robot1, rmodel1, rdata1, gmodel1, gdata1 = initSolo(solo=False)
robot2, rmodel2, rdata2, gmodel2, gdata2 = initSolo(solo=False, free_flyer=True)

robot0.initViewer(loadModel=True, sceneName="solo0")
#robot2.initViewer(loadModel=True)
robot1.initViewer(loadModel=True, sceneName="solo1")
robot2.initViewer(loadModel=True, sceneName="solo2")

robot2.displayCollisions(True)
robot2.displayVisuals(False)
robot2_translation = np.array([0,0.,0])

showLinks_1 = ['HL_U','HL_L', 'HL_F','HL_S']
#showLinks_1 = ['FL_U','FL_L', 'FL_F','FL_S',\
#                'FR_U','FR_L', 'FR_F','FR_S']

# Common to all scenes
gv = robot2.viewer.gui
win_id = gv.getWindowList()[0]

gv.setBackgroundColor1(win_id, [0.75,0.75,0.75,1.])
gv.setBackgroundColor2(win_id, [0.85,0.85,0.85,1.])


for n in getSceneNodes(gv, 'solo0'):
    gv.setColor(n, [0.4,0.4,0.4,1])

for n in getSceneNodes(gv, 'solo1'):
    gv.setColor(n, [0.2,1,0.5,0.8])
    if not checkLinkList(n, showLinks_1):
        gv.setColor(n, [0.8,0.2,0.2,0.])

for n in getSceneNodes(gv, 'solo2'):
    gv.setColor(n, [0.8,0.5,0.2,0.6])
    if('base' in n) or ('SHOULDER' in n):
        gv.setVisibility(n, 'OFF')



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

# Select timestamps in logs and framerate
start_time = 000
length_time = 22000
downsample = 20

saved_cam_transform = [1.1111056804656982, -0.3006046414375305, 0.4920024871826172, 0.4313039779663086, 0.289389431476593, 0.4372677803039551, 0.7341849207878113]
saved_cam_transform_2 = [1.8467745780944824, 1.3208256959915161, 1.0858362913131714, 0.25357887148857117, 0.4054725170135498, 0.7733896374702454, 0.4161228537559509]
saved_cam_transform_3 = [0.9342212080955505, 0.7453151941299438, 0.09584774076938629, 0.28465965390205383, 0.5007022023200989, 0.7287548780441284, 0.37038156390190125]

saved_cam_transform  =[0.2753353714942932, -0.6551246643066406, 0.4682965576648712, 0.49102744460105896, 0.06779501587152481, 0.1611940860748291, 0.8534122109413147]

gv.setCameraTransform(win_id, ref_cam_transform)

# Lerp translation of capsules model (gadget)
transl_start = 6000
transl_range = [0,0.4]
transl_length = 4000

caps_fade_start = 4000
caps_fade_range = [[0.8,0.5,0.2,1.], [1.,1.,1.,0.]]
caps_fade_length = 4000

cam_lerp_0 = [6000, 4000, [ref_cam_transform, saved_cam_transform]]
cam_lerp_1 = [14000, 4000, [saved_cam_transform, ref_cam_transform]]

rmodel2 = robot2.model
rdata2 = rmodel2.createData()

# Collision data parameters
view_coll_data = True
dist_thresh = 0.05

distToPlot = []
figure_size = (9,9)
plt.figure(figsize=figure_size)

# Shoulder viz
plots = []
shd_plots = [[]]*4
shd_dist_landscape = np.load('/home/tnoel/stage/solo-collisions/src/python/ref_net3d_dist_landscape.npy', allow_pickle=True)
view_shoulder_data = False

for k in range(length_time):
    q = q_ref[start_time + k][7:]
    #vq = vq_ref[start_time + k][6:]

    mes_q = q_log[start_time + k]
    #mes_vq = vq_log[start_time + k]

    #robot2_translation[1] = lerp_value(transl_range, transl_length, k, transl_start)

    robot2_translation = lerp_vector([[0,0,0],[0,0.,0.]], transl_length, k, transl_start)
    mes_q_copy = np.zeros(19)
    mes_q_copy[0:3] = robot2_translation
    mes_q_copy[3:7] = [0,0,0,0]
    mes_q_copy[7:] = mes_q

    if view_coll_data:
        dist_legs, Jlegs, wp_legs = getLegsCollResults(mes_q)
        
        distToPlot.append(dist_legs)
        if(len(distToPlot) > 200):
            distToPlot.pop(0)

    if view_shoulder_data :
        dist_shd, Jshd = getAllShouldersCollisionsResults(mes_q, nnCCollFun, 3, offset=0.11) #offset with 3 inputs: 0.18 (small), 0.11 (large)"

    if(k%downsample == 0):  
        #print(len(plots))
        if(view_shoulder_data):
            knee_slice = 0 
            ind = int(len(shd_dist_landscape)*((q[7 + knee_slice*3 + 2]%(2*np.pi))/(2*np.pi)))
            local_dist_landscape = shd_dist_landscape[ind].copy()
            
            #visualizeShoulderBackground(mes_q_copy, shd_dist_landscape, 0.2, dim=3)
            #plots.append(plt.imshow(local_dist_landscape))
            if(k%(10*downsample)==0):
                visualizeShoulderBackground(mes_q_copy, shd_dist_landscape, 0.2, dim=3)
            visualizeShouldersCollisions(shd_plots, mes_q_copy, dist_shd, 0.2, dim=3)
            if (len(plots) > 1):
                plots.pop().remove()
        
        caps_alpha = lerp_value([0.,0.5], caps_fade_length, k, caps_fade_start)
        for n in getSceneNodes(gv, 'solo2'):
            gv.setColor(n, [0.8,0.5,0.2,caps_alpha])
            #gv.setColor(n, [0.8,0.5,0.2,1])
        
        cam_transform = lerp_transform(cam_lerp_0[2], cam_lerp_0[1], k, cam_lerp_0[0])
        #if(k>cam_lerp_0[0]):
            
        #    cam_transform = lerp_transform(cam_lerp_1[2], cam_lerp_1[1], k, cam_lerp_1[0])
        gv.setCameraTransform(win_id, cam_transform)
        
        if(view_coll_data):
            
            distPlot = plotPairsDist(np.array(distToPlot), caps_frames_list)
            plots.append(distPlot)
            if (len(plots) > 1):
                for p in plots[0]:
                    p.pop(0).remove()
                plots.pop(0)

            if (k > 2000):
                addSceneMultiplePairsWitness(gv, 'solo0', rmodel0, rdata0, mes_q, caps_frames_list, wp_legs, dist_legs, translation=np.array(robot2_translation))
            
        robot0.display(mes_q)
        robot1.display(q)
        robot2.display(mes_q_copy)
        
        gv.captureFrame(win_id, 'solo_viewer_images/test_{:05d}.png'.format(k))
        #plt.savefig('/home/tnoel/solo_fig_images/test_fig_{:05d}'.format(k))
        #time.sleep(downsample*1e-3)
        #plt.pause(downsample*1e-3)
        
        gv.refresh()

#plt.show()