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
#robot_config[8] = np.pi/2
#relativePlacement(rmodel, base_link, fl_upper_leg)

baseMflul = relativePlacement(rmodel, rdata, robot_config, base_link, fl_upper_leg)

Jf1 = pio.computeFrameJacobian(rmodel, rdata, robot_config, fl_upper_leg)
Jf2 = pio.computeFrameJacobian(rmodel, rdata, robot_config, fr_upper_leg)

Jf1_transl, Jf1_rot = Jf1[:3,6:], Jf1[3:,6:]
Jf2_transl, Jf2_rot = Jf2[:3,6:], Jf2[3:,6:]

#print(Mbase)
#print(Mflul)
np.set_printoptions(precision=3)
#print(Jf2_transl - Jf1_transl)
print("Translation")
print(Jf1_transl - Jf2_transl)
print("Rotation")
print(Jf1_rot)