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


def computeVelJacobian(model, data, config, fInd1, fInd2, p1, p2):
    # Compute frames placements
    oMf1 = relativePlacement(model, data, config, model.getFrameId("base_link"), fInd1)
    oMf2 = relativePlacement(model, data, config, model.getFrameId("base_link"), fInd2)
    # Frames jacobians
    f1Jf1 = pio.computeFrameJacobian(model, data, config, fInd1, pio.LOCAL)
    f2Jf2 = pio.computeFrameJacobian(model, data, config, fInd2, pio.LOCAL)
    # get rid of free floating base coord.
    f1Jf1 = f1Jf1[:,6:] 
    f2Jf2 = f2Jf2[:,6:]
    # Compute local velocities
    f1Jp1 = f1Jf1[:3,:] + pio.skew(p1)@f1Jf1[3:,:]
    f2Jp2 = f2Jf2[:3,:] + pio.skew(p2)@f2Jf2[3:,:]
    # Compute world vel.
    oJp1 = oMf1.rotation @ f1Jp1
    oJp2 = oMf2.rotation @ f2Jp2

    return oJp1, oJp2


def computeVelJacobianFiniteDiff(model, data, config, fInd1, fInd2, p1, p2, dqi = 0.01):
    # Compute frames placements
    oMf1 = relativePlacement(model, data, config, model.getFrameId("base_link"), fInd1)
    oMf2 = relativePlacement(model, data, config, model.getFrameId("base_link"), fInd2)
    # Initialize jacobian and dq vector
    oJp1 = np.zeros((len(p1),12))
    dq = np.zeros(19)
    for i in range(12):
        # Compute finite difference for frame i (index : 7+i to account for free floating base in config vector)
        dq[7 + i] = dqi
        oMf1_add = relativePlacement(model, data, config + dq, model.getFrameId("base_link"), fInd1)
        # Compute jacobian col. as pos difference and rescale for dqi
        oJip1 = (oMf1_add.act(p1) - oMf1.act(p1))/dqi
        oJp1[:,i] = oJip1
        # Reset dq vector
        dq[7 + i] = 0
    return oJp1


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
p1 = np.array([0,0,0.])
p2 = np.array([0,0,-0.1])

oJp1, oJp2 = computeVelJacobian(rmodel, rdata, robot_config, fl_lower_leg, fr_upper_leg, p1, p2)
oJp1_fd = computeVelJacobianFiniteDiff(rmodel, rdata, robot_config, fl_lower_leg, fr_upper_leg, p1, p2, dqi=0.01)

#np.set_printoptions(precision=3)
print("oVp1 : ")
print(oJp1)
print("oVp1 finite diff : ")
print(oJp1_fd)
print("Max. Err :")
print(np.max(abs(oJp1_fd - oJp1)))