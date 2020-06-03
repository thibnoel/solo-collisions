import pinocchio as pio
from example_robot_data import loadSolo
import hppfcl
import numpy as np
import matplotlib.pyplot as plt
import time
from numpy.linalg import norm
from solo12_legs_collisions_utils import initSolo, addCapsule

robot, rmodel, rdata, gmodel, gdata = initSolo()
robot_config = np.zeros(robot.q0.shape) # Default configuration
# Initialize the gepetto-gui viewer
enableGUI = True

if(enableGUI):
        robot.initViewer(loadModel=True)
        gv = robot.viewer.gui
        # Display the robot
        robot.rebuildData() 
        robot.displayCollisions(True)
        robot.displayVisuals(False)
        robot.display(robot_config) 
        for n in gv.getNodeList():
                if 'collision' in n and 'simple' not in n and len(n)>27:
                        gv.setVisibility(n,'ON')
                        gv.setColor(n, [1,1,1,1])
        gv.refresh()

# Links to add capsules to
fl_upper_leg = rmodel.getFrameId("FL_UPPER_LEG")
fl_lower_leg = rmodel.getFrameId("FL_LOWER_LEG")
fr_upper_leg = rmodel.getFrameId("FR_UPPER_LEG")
fr_lower_leg = rmodel.getFrameId("FR_LOWER_LEG")
hl_upper_leg = rmodel.getFrameId("HL_UPPER_LEG")
hl_lower_leg = rmodel.getFrameId("HL_LOWER_LEG")
hr_upper_leg = rmodel.getFrameId("HR_UPPER_LEG")
hr_lower_leg = rmodel.getFrameId("HR_LOWER_LEG")
# Associated joints
fl_hfe = rmodel.getJointId("FL_HFE")
fl_kfe = rmodel.getJointId("FL_KFE")
fr_hfe = rmodel.getJointId("FR_HFE")
fr_kfe = rmodel.getJointId("FR_KFE")
hl_hfe = rmodel.getJointId("HL_HFE")
hl_kfe = rmodel.getJointId("HL_KFE")
hr_hfe = rmodel.getJointId("HR_HFE")
hr_kfe = rmodel.getJointId("HR_KFE")

CAPS_LENGTH = 0.2
CAPS_RADIUS = 0.02

upper_caps_pos_offset = np.array([0.,0.0,0])
lower_caps_pos_offset = np.array([0,-0.0,0])

upper_offset = pio.SE3(np.eye(3), upper_caps_pos_offset + np.array([0,0,-CAPS_LENGTH*0.5]))
lower_offset = pio.SE3(np.eye(3), lower_caps_pos_offset + np.array([0,0,-CAPS_LENGTH*0.5]))

fl_upper_caps = addCapsule("FL_UPPER_simple_caps",CAPS_LENGTH,CAPS_RADIUS,fl_upper_leg, fl_hfe, upper_offset, rmodel, gmodel, gui=gv)
fl_lower_caps = addCapsule("FL_LOWER_simple_caps",CAPS_LENGTH,CAPS_RADIUS,fl_lower_leg, fl_kfe, lower_offset, rmodel, gmodel, gui=gv)

fr_upper_caps = addCapsule("FR_UPPER_simple_caps",CAPS_LENGTH,CAPS_RADIUS,fr_upper_leg, fr_hfe, lower_offset, rmodel, gmodel, gui=gv)
hl_upper_caps = addCapsule("HL_UPPER_simple_caps",CAPS_LENGTH,CAPS_RADIUS,hl_lower_leg, hl_kfe, upper_offset, rmodel, gmodel, gui=gv)

gmodel.addCollisionPair(pio.CollisionPair(fl_upper_caps,hl_upper_caps))
gmodel.addCollisionPair(pio.CollisionPair(fl_upper_caps,fl_lower_caps))

robot_config[6] = 0.
robot_config[8] = 0
robot_config[9] = 0.
robot_config[10] = 0
robot_config[11] = 0.
robot_config[12] = 0
robot_config[13] = 1.
robot_config[14] = 0
robot_config[15] = 0.
robot_config[16] = 0
robot_config[17] = 0.
robot_config[18] = 0. 

gdata  = gmodel.createData()
pio.computeDistances(rmodel,rdata,gmodel,gdata, robot_config)
collisions_dist = gdata.distanceResults

print(rmodel.nq)
print(collisions_dist[0].min_distance, collisions_dist[1].min_distance)

# Display the robot
robot.rebuildData() 
robot.displayCollisions(True)
robot.displayVisuals(False)
robot.display(robot_config) 
for n in gv.getNodeList():
        if 'collision' in n and 'simple' not in n and len(n)>27:
                gv.setVisibility(n,'ON')
gv.refresh()