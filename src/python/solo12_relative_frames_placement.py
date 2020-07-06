import pinocchio as pio
from example_robot_data import loadSolo
import numpy as np
import matplotlib.pyplot as plt
import time
from solo12_legs_collisions_utils import initSolo

robot, rmodel, rdata, gmodel, gdata = initSolo()
robot_config = np.zeros(robot.q0.shape) # Default configuration

def relativePlacement(rmodel, rdata, config, frameIndex1, frameIndex2):
        pio.framesForwardKinematics(rmodel, rdata, config)

        oMf1 = rdata.oMf[frameIndex1]
        oMf2 = rdata.oMf[frameIndex2]

        return np.linalg.inv(oMf1) @ oMf2

# Get the robot frames we're interested in
fl_upper_leg = rmodel.getFrameId("FL_UPPER_LEG")
fl_lower_leg = rmodel.getFrameId("FL_LOWER_LEG")
fr_upper_leg = rmodel.getFrameId("FR_UPPER_LEG")
fr_lower_leg = rmodel.getFrameId("FR_LOWER_LEG")
hl_upper_leg = rmodel.getFrameId("HL_UPPER_LEG")
hl_lower_leg = rmodel.getFrameId("HL_LOWER_LEG")
hr_upper_leg = rmodel.getFrameId("HR_UPPER_LEG")
hr_lower_leg = rmodel.getFrameId("HR_LOWER_LEG")
base_link = rmodel.getFrameId("base_link")

#robot_config = np.random.random(robot_config.shape)
#robot_config[7] = np.pi/2
#relativePlacement(rmodel, base_link, fl_upper_leg)

baseMflul = relativePlacement(rmodel, rdata, robot_config, base_link, fl_upper_leg)

# p1 coord. in f1, p2 in f2
p1 = np.array([0,0,0.1])
p2 = np.array([0,0,-0.1])

# Frames jacobians
f1Jf1 = pio.computeFrameJacobian(rmodel, rdata, robot_config, fl_upper_leg, pio.LOCAL)
f2Jf2 = pio.computeFrameJacobian(rmodel, rdata, robot_config, fr_upper_leg, pio.LOCAL)

f1Jf1 = f1Jf1[:,6:] # get rid of free floating base coord.
f2Jf2 = f2Jf2[:,6:] # get rid of free floating base coord.c

f1Vp1 = f1Jf1[:3,:] + pio.skew(p1)@f1Jf1[3:,:]
f2Vp2 = f2Jf2[:3,:] + pio.skew(p2)@f2Jf2[3:,:]

oMf1 = relativePlacement(rmodel, rdata, robot_config, base_link, fl_upper_leg)
oMf2 = relativePlacement(rmodel, rdata, robot_config, base_link, fr_upper_leg)

oVp1 = np.array([pio.SE3(oMf1).act(f1Vp1[:,i]) for i in range(12)]).T
oVp2 = np.array([pio.SE3(oMf2).act(f2Vp2[:,i]) for i in range(12)]).T

#print(Mbase)
#print(Mflul)
np.set_printoptions(precision=3)
#print(Jf2_transl - Jf1_transl)
print("oVp1")
print(oVp1)
print("f2Vp2")
print(oVp2)

print("Diff :")
print(oVp2 - oVp1)