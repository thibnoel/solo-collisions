import pinocchio as pio
from example_robot_data import loadSolo
import numpy as np
import matplotlib.pyplot as plt
import time
import hppfcl


def relativePlacement(rmodel, rdata, config, frameIndex1, frameIndex2):
    pio.framesForwardKinematics(rmodel, rdata, config)
    oMf1 = rdata.oMf[frameIndex1]
    oMf2 = rdata.oMf[frameIndex2]

    return pio.SE3(oMf1).inverse() * pio.SE3(oMf2)


def computeVelJacobian(model, data, config, fInd1, fInd2, f1_p1, f2_p2, floatingBase=True):
    # Compute frames placements
    oMf1 = relativePlacement(model, data, config, model.getFrameId("base_link"), fInd1)
    oMf2 = relativePlacement(model, data, config, model.getFrameId("base_link"), fInd2)

    # Frames jacobians
    f1Jf1 = pio.computeFrameJacobian(model, data, config, fInd1, pio.LOCAL)
    f2Jf2 = pio.computeFrameJacobian(model, data, config, fInd2, pio.LOCAL)
    
    # get rid of free floating base coord.
    if(floatingBase):
        f1Jf1 = f1Jf1[:,6:] 
        f2Jf2 = f2Jf2[:,6:]
    # Compute local velocities
    f1Jp1 = f1Jf1[:3,:] - pio.skew(f1_p1)@f1Jf1[3:,:]
    f2Jp2 = f2Jf2[:3,:] - pio.skew(f2_p2)@f2Jf2[3:,:]
    
    # Compute world vel.
    oJp1 = oMf1.rotation @ f1Jp1
    oJp2 = oMf2.rotation @ f2Jp2
    
    return oJp1, oJp2

def computeDistJacobian(model, data, config, fInd1, fInd2, o_p1, o_p2, floatingBase=True):
    oMf1 = relativePlacement(model, data, config, model.getFrameId("base_link"), fInd1)
    oMf2 = relativePlacement(model, data, config, model.getFrameId("base_link"), fInd2)
    # Compute vel jacobian
    #f1_p1 = -oMf1.translation + np.linalg.inv(oMf1.rotation)@o_p1
    #f2_p2 = -oMf2.translation + np.linalg.inv(oMf2.rotation)@o_p2

    f1_p1 = oMf1.inverse().translation + oMf1.inverse().rotation@o_p1
    f2_p2 = oMf2.inverse().translation + oMf2.inverse().rotation@o_p2

    oJp1, oJp2 = computeVelJacobian(model, data, config, fInd1, fInd2, f1_p1, f2_p2, floatingBase=floatingBase)
    oJ_Dp = oJp2 - oJp1
    
    Dp = ((oMf2.translation + oMf2.rotation@f2_p2)-(oMf1.translation + oMf1.rotation@f1_p1))
    dist = np.sqrt(Dp.T@Dp)
    #Dp = Dp/dist

    Jdist = Dp.T@oJ_Dp/dist

    return Jdist

def computeVelJacobianFiniteDiff(model, data, config, fInd1, fInd2, p1, p2, dqi = 0.01):
    # Compute frames placements
    oMf1 = relativePlacement(model, data, config, model.getFrameId("base_link"), fInd1)
    oMf2 = relativePlacement(model, data, config, model.getFrameId("base_link"), fInd2)
    # Initialize jacobian and dq vector
    oJp1 = np.zeros((len(p1),12))
    oJp2 = np.zeros((len(p2),12))
    dq = np.zeros(19)
    for i in range(12):
        # Compute finite difference for qi (index : 7+i to account for free floating base in config vector)
        dq[7 + i] = dqi

        oMf1_add = relativePlacement(model, data, config + dq, model.getFrameId("base_link"), fInd1)
        # Compute jacobian col. as pos difference and rescale for dqi
        oJip1 = ((oMf1_add.translation + oMf1_add.rotation@p1) - (oMf1.translation + oMf1.rotation@p1))/dqi
        oJp1[:,i] = oJip1

        oMf2_add = relativePlacement(model, data, config + dq, model.getFrameId("base_link"), fInd2)
        # Compute jacobian col. as pos difference and rescale for dqi
        oJip2 = ((oMf2_add.translation + oMf2_add.rotation@p2) - (oMf2.translation + oMf2.rotation@p2))/dqi
        oJp2[:,i] = oJip2

        # Reset dq vector
        dq[7 + i] = 0

    return oJp1, oJp2

def computeDistJacobianFiniteDiff(model, data, config, fInd1, fInd2, p1, p2, dqi = 0.01, floatingBase=True):
    # Compute frames placements
    oMf1 = relativePlacement(model, data, config, model.getFrameId("base_link"), fInd1)
    oMf2 = relativePlacement(model, data, config, model.getFrameId("base_link"), fInd2)

    # Initialize jacobian and dq vector
    Jdist = np.zeros(len(config))
    
    dq = np.zeros(len(config) + 7) if floatingBase else np.zeros(len(config))

    #p1 = -oMf1.translation + np.linalg.inv(oMf1.rotation)@p1
    #p2 = -oMf2.translation + np.linalg.inv(oMf2.rotation)@p2
    p1 = oMf1.inverse().translation + oMf1.inverse().rotation@p1
    p2 = oMf2.inverse().translation + oMf2.inverse().rotation@p2

    for i in range(len(config)):
        # Compute finite difference for frame i (index : 7+i to account for free floating base in config vector)
        q_index = 7 + i if floatingBase else i
        dq[q_index] = dqi

        oMf1_add = relativePlacement(model, data, config + dq, model.getFrameId("base_link"), fInd1)
        oMf2_add = relativePlacement(model, data, config + dq, model.getFrameId("base_link"), fInd2)

        Dp = ((oMf2.translation + oMf2.rotation@p2)-(oMf1.translation + oMf1.rotation@p1))
        Dp_add = ((oMf2_add.translation + oMf2_add.rotation@p2)-(oMf1_add.translation + oMf1_add.rotation@p1))
        

        dist = np.sqrt(Dp.T@Dp)
        dist_add = np.sqrt(Dp_add.T@Dp_add)

        Jdist[i] = (dist_add - dist)/dqi

        # Reset dq vector
        dq[q_index] = 0
    
    return Jdist


def getCollisionResults(q, rmodel, rdata, gmodel, gdata):
    pio.computeDistances(rmodel,rdata,gmodel,gdata, q)
    collisions_dist = gdata.distanceResults
    return collisions_dist


def compute_legs_Jdist_avoidance(q, rmodel, rdata, gmodel, gdata, FD=False):
    counter = 0
    collisions_result = getCollisionResults(q, rmodel, rdata, gmodel, gdata)
    Jdist = []
    dist_vec = []
    pairs = []
    wpoints = []

    q_des_coll = np.zeros(len(q))
    
    while(counter<len(collisions_result)):
        collDist = collisions_result[counter].min_distance
        p1 = collisions_result[counter].getNearestPoint1()
        p2 = collisions_result[counter].getNearestPoint2()

        #print(p1,p2)

        frame1 = gmodel.geometryObjects[gmodel.collisionPairs[counter].first].parentFrame
        frame2 = gmodel.geometryObjects[gmodel.collisionPairs[counter].second].parentFrame

        if(FD):
            Jlocal_dist = computeDistJacobianFiniteDiff(rmodel, rdata, q, frame1, frame2, p1, p2, floatingBase=False)
        else:
            Jlocal_dist = computeDistJacobian(rmodel, rdata, q, frame1, frame2, p1, p2, floatingBase=False)
        
        #if(counter==3):
        #    print("Not FD")
        #    print(computeDistJacobian(rmodel, rdata, q, frame1, frame2, p1, p2, floatingBase=False))

        dist_vec.append(collDist)
        Jdist.append(Jlocal_dist)
        wpoints.append([p1,p2])
        
        pairs.append(gmodel.collisionPairs[counter])
            
        counter += 1
    return dist_vec, Jdist, pairs, wpoints