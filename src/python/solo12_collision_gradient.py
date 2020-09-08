import pinocchio as pio
from example_robot_data import loadSolo
import numpy as np
import matplotlib.pyplot as plt
import time
from solo12_legs_collisions_utils import initSolo


def relativePlacement(rmodel, rdata, config, frameIndex1, frameIndex2):
    pio.framesForwardKinematics(rmodel, rdata, config)
    oMf1 = rdata.oMf[frameIndex1]
    oMf2 = rdata.oMf[frameIndex2]

    return pio.SE3(oMf1).inverse() * pio.SE3(oMf2)


def computeVelJacobian(model, data, config, fInd1, fInd2, p1, p2, floatingBase=True):
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
    f1Jp1 = f1Jf1[:3,:] - pio.skew(p1)@f1Jf1[3:,:]
    f2Jp2 = f2Jf2[:3,:] - pio.skew(p2)@f2Jf2[3:,:]
    
    # Compute world vel.
    oJp1 = oMf1.rotation @ f1Jp1
    oJp2 = oMf2.rotation @ f2Jp2
    
    return oJp1, oJp2

def computeDistJacobian(model, data, config, fInd1, fInd2, p1, p2, floatingBase=True):
    oMf1 = relativePlacement(model, data, config, model.getFrameId("base_link"), fInd1)
    oMf2 = relativePlacement(model, data, config, model.getFrameId("base_link"), fInd2)
    # Compute vel jacobian
    oJp1, oJp2 = computeVelJacobian(model, data, config, fInd1, fInd2, p1, p2, floatingBase=floatingBase)
    oJ_Dp = oJp2 - oJp1
    
    Dp = ((oMf2.translation + oMf2.rotation@p2)-(oMf1.translation + oMf1.rotation@p1))
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
    Jdist = np.zeros(12)
    
    dq = np.zeros(19) if floatingBase else np.zeros(12)

    for i in range(12):
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


##### MAIN #####
if __name__ == "__main__":
    # Model initialization
    robot, rmodel, rdata, gmodel, gdata = initSolo()
    robot_config = np.zeros(robot.q0.shape) # Default configuration
    robot_config = np.random.random(robot.q0.shape)

    # Get frames indices
    fl_upper_leg = rmodel.getFrameId("FL_UPPER_LEG")
    fl_lower_leg = rmodel.getFrameId("FL_LOWER_LEG")
    fr_upper_leg = rmodel.getFrameId("FR_UPPER_LEG")
    fr_lower_leg = rmodel.getFrameId("FR_LOWER_LEG")
    hl_upper_leg = rmodel.getFrameId("HL_UPPER_LEG")
    hl_lower_leg = rmodel.getFrameId("HL_LOWER_LEG")
    hr_upper_leg = rmodel.getFrameId("HR_UPPER_LEG")
    hr_lower_leg = rmodel.getFrameId("HR_LOWER_LEG")
    base_link = rmodel.getFrameId("base_link")

    # p1 coord. in f1, p2 in f2
    p1 = np.array([0.1,0,0.])
    p2 = np.array([0,0.1,-0.0])

    oJp1, oJp2 = computeVelJacobian(rmodel, rdata, robot_config, fl_lower_leg, fr_lower_leg, p1, p2)
    oJp1_fd, oJp2_fd = computeVelJacobianFiniteDiff(rmodel, rdata, robot_config, fl_lower_leg, fr_lower_leg, p1, p2, dqi=0.001)

    oJ_Dp = oJp2 - oJp1
    oJ_Dp_fd = oJp2_fd - oJp1_fd

    Jdist = computeDistJacobian(rmodel, rdata, robot_config, fl_upper_leg, fr_upper_leg, p1, p2)
    Jdist_fd = computeDistJacobianFiniteDiff(rmodel, rdata, robot_config, fl_upper_leg, fr_upper_leg, p1, p2, dqi=1e-2)

    #np.set_printoptions(precision=3)
    print("oV_p1 : ")
    print(oJp1)
    print("oV_p1 finite diff : ")
    print(oJp1_fd)
    print("Max. Err (p1) :")
    print(np.max(abs(oJp1_fd - oJp1)))
    print("Max. Err (p2) :")
    print(np.max(abs(oJp2_fd - oJp2)))
    print("Max. Err (Dp) :")
    print(np.max(abs(oJ_Dp_fd - oJ_Dp)))
    print("Jdist : ")
    print(Jdist)
    print("Jdist finite diff : ")
    print(Jdist_fd)
    print("Max. Err (Jdist) :")
    print(np.max(abs(Jdist_fd - Jdist)))