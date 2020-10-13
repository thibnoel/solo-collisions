import pinocchio as pio
import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt

from solo_collisions_avoidance_control.solo_initialization import *

def initViewer(robot_wrapper, init_config, coll=True):
    robot_wrapper.initViewer(loadModel=True)
    gv = robot_wrapper.viewer.gui
    # Display the robot
    robot_wrapper.rebuildData() 
    robot_wrapper.displayCollisions(coll)
    robot_wrapper.displayVisuals(True)
    robot_wrapper.display(init_config) 
    
    for n in gv.getNodeList():
        if 'LEG' in n and 'collision' in n and len(n)>27:
            #gv.setVisibility(n,'OFF')
            gv.setColor(n, [1,0.5,0,0.1])
    
    gv.refresh()
    
    return gv


def visualizeCollisionDist(gv, p1, p2, name, color, init=False):
    ### --- display witness as normal patch tangent to capsule
    if(init):
        for i in range(2):
                gv.addCylinder('world/pinocchio/collisions/simple_patch_' + name + '_%d'%i, .01, .003, color)
        gv.addLine('world/pinocchio/collisions/line_' + name, p1.tolist(), p2.tolist(), color)

    direc = (p2-p1)/norm(p2-p1) 

    M1 = pio.SE3(pio.Quaternion.FromTwoVectors(np.matrix([0,0,1]).T,p1-p2).matrix(),p1)
    M2 = pio.SE3(pio.Quaternion.FromTwoVectors(np.matrix([0,0,1]).T,p2-p1).matrix(),p2)
    gv.applyConfiguration('world/pinocchio/collisions/simple_patch_' + name + '_0',pio.SE3ToXYZQUATtuple(M1))
    gv.applyConfiguration('world/pinocchio/collisions/simple_patch_' + name + '_1',pio.SE3ToXYZQUATtuple(M2))
    gv.setLineExtremalPoints('world/pinocchio/collisions/line_' + name, p1.tolist(), p2.tolist())

    gv.setColor('world/pinocchio/collisions/simple_patch_' + name + '_0', color)
    gv.setColor('world/pinocchio/collisions/simple_patch_' + name + '_1', color)
    gv.setColor('world/pinocchio/collisions/line_' + name, color)

    gv.refresh()


def visualizePair(gv, rmodel, rdata, q, caps_frames, local_wpoints, color, world_frame=False, init=False):
    pio.forwardKinematics(rmodel, rdata, q)
    pio.updateFramePlacements(rmodel, rdata)

    leg_seg0 = rmodel.getFrameId(caps_frames[0])
    leg_seg1 = rmodel.getFrameId(caps_frames[1])
    if(not world_frame):
        p0 = rdata.oMf[leg_seg0].rotation@local_wpoints[0] + rdata.oMf[leg_seg0].translation
        p1 = rdata.oMf[leg_seg1].rotation@local_wpoints[1] + rdata.oMf[leg_seg1].translation
    else:
        p0 = local_wpoints[0]
        p1 = local_wpoints[1]

    #print(p0,p1)
    visualizeCollisionDist(gv, np.array(p0), np.array(p1), caps_frames[0] + '_' + caps_frames[1], color, init=init)


def visualizeCollisions(gv, rmodel, rdata, q, caps_frames_list, legs_dist_list, wpoints_list, viz_thresh, activation_thresh, init=False):
    for i in range(len(caps_frames_list)):
        color = [0,0,0,0]
        if(legs_dist_list[i]) < viz_thresh:
            color = [0,1,0,1] if legs_dist_list[i] > activation_thresh else [1,0,0,1]
        visualizePair(gv, rmodel, rdata, q, caps_frames_list[i], wpoints_list[i], color, world_frame=False, init=init)


def visualizeTorques(gv, rmodel, rdata, tau_q, init=False):
    for k in range(len(tau_q)):
        jointFrame = rdata.oMi[k]
        name = 'world/pinocchio/collisions/torque_' + str(k)
        color = [0,0,1,1]
        dir = 1 if tau_q[k]>0 else -1
        orientation = pio.SE3(pio.Quaternion.FromTwoVectors(np.matrix([1,0,0]).T,np.matrix([0,dir,0]).T).matrix(),jointFrame.translation)

        if(init):
            gv.addArrow(name, 0.003, 1, color)
        gv.resizeArrow(name, 0.003, np.abs(tau_q[k]))
        gv.applyConfiguration(name, pio.SE3ToXYZQUATtuple(orientation))


'''
solo_coll, rmodel, rdata, gmodel, gdata = initSolo(True)

p1 = np.array([0.0, 0.0, 0.])
p2 = np.array([0.0, 0.0, 0.])
color = [0,1,0,1]
name="test_coll" 

q = np.zeros(8)
q[0] = 1

caps_frames = ["FL_LOWER_LEG", "HL_LOWER_LEG"]

viewer_coll = initViewer(solo_coll, np.zeros(8), coll=True)
#visualizeCollisionDist(viewer_coll, p1, p2, name, color)

for k in range(100):
    visualizePair(viewer_coll, rmodel, rdata, q, caps_frames, [p1 + np.array([0,0,0.01*k]),p2], color)
solo_coll.display(q)
'''