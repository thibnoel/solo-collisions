import pinocchio as pio
from example_robot_data import loadSolo
import hppfcl
import numpy as np
import matplotlib.pyplot as plt
import time
import capsule_approximation as capsApprox

from numpy.linalg import norm
from solo12_legs_collisions_utils import initSolo, addCapsule


def getCapsuleApprox(solo_meshfile):
        solo_description = "/opt/openrobots/share/example-robot-data/robots/solo_description/"
        urdf_filename = solo_description + "robots/solo12.urdf"
        mesh_filename = solo_description + "meshes/obj/with_foot/" + solo_meshfile

        mesh_loader = hppfcl.MeshLoader()
        mesh = mesh_loader.load(mesh_filename, np.ones(3))
        num_vertices = mesh.num_vertices
        vertices = np.array([mesh.vertices(i) for i in range(num_vertices)])

        a, b, r = capsApprox.capsule_approximation(vertices)

        return a, b, np.linalg.norm(b-a), r


def capsApproxToSE3(a,b, reverse=False):
        translation = (a+b)*0.5
        if(reverse):
                translation[0]*=-1
                translation[1]*=-1
        capsDir = ((b-a)/np.linalg.norm(b-a)).reshape(3)
        up = (np.array([0,0,1])).reshape(3)
        v = np.cross(up, capsDir)
        if(reverse):
                v = np.cross(capsDir, up)
        c = np.dot(up, capsDir)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))

        return pio.SE3(rotation_matrix, translation)

def visualizeCollisionDist(p1, p2, name, color):
        gv.addSphere("world/" + name + "/p1", .1, color)
        gv.addSphere("world/" + name + "/p2", .1, color)
        gv.applyConfiguration("world/" + name + "/p1", p1.tolist()+[0,0,0,0])
        gv.applyConfiguration("world/" + name + "/p2", p2.tolist()+[0,0,0,0])
        
        gv.refresh()


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
base_link = rmodel.getFrameId("base_link")
base_link_geom = gmodel.getGeometryId("base_link_0")
fl_upper_leg = rmodel.getFrameId("FL_UPPER_LEG")
fl_upper_leg_geom = gmodel.getGeometryId("FL_UPPER_LEG_0")
fl_lower_leg = rmodel.getFrameId("FL_LOWER_LEG")
fl_lower_leg_geom = gmodel.getGeometryId("FL_LOWER_LEG_0")
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

# Capsules approx for legs segments meshes
L_upperCapsApprox = getCapsuleApprox("solo_upper_leg_left_side.obj")
R_lowerCapsApprox = getCapsuleApprox("solo_lower_leg_right_side.obj")

UPPER_CAPS_LENGTH = L_upperCapsApprox[2] 
UPPER_CAPS_RADIUS = L_upperCapsApprox[3] 

LOWER_CAPS_LENGTH = R_lowerCapsApprox[2] 
LOWER_CAPS_RADIUS = R_lowerCapsApprox[3] 

l_upper_offset = capsApproxToSE3(L_upperCapsApprox[0], L_upperCapsApprox[1])
r_upper_offset = capsApproxToSE3(L_upperCapsApprox[0], L_upperCapsApprox[1], reverse=True)
l_lower_offset = capsApproxToSE3(R_lowerCapsApprox[0], R_lowerCapsApprox[1], reverse=True)
r_lower_offset = capsApproxToSE3(R_lowerCapsApprox[0], R_lowerCapsApprox[1])


print("Capsules approx. :")
print("Upper leg seg. : \nLength = {:.4f}, \nRadius = {:.4f}, \nM : {}".format(UPPER_CAPS_LENGTH, UPPER_CAPS_RADIUS, l_upper_offset))
print("\nLower leg seg. : \nLength = {:.4f}, \nRadius = {:.4f}, \nM : {}".format(LOWER_CAPS_LENGTH, LOWER_CAPS_RADIUS, l_lower_offset))

fl_upper_caps = addCapsule("FL_UPPER_simple_caps",UPPER_CAPS_LENGTH,UPPER_CAPS_RADIUS,fl_upper_leg, fl_hfe, l_upper_offset, rmodel, gmodel, gui=gv)
fl_lower_caps = addCapsule("FL_LOWER_simple_caps",LOWER_CAPS_LENGTH,LOWER_CAPS_RADIUS,fl_lower_leg, fl_kfe, l_lower_offset, rmodel, gmodel, gui=gv)
fr_upper_caps = addCapsule("FR_UPPER_simple_caps",UPPER_CAPS_LENGTH,UPPER_CAPS_RADIUS,fr_upper_leg, fr_hfe, r_upper_offset, rmodel, gmodel, gui=gv)
fr_lower_caps = addCapsule("FR_LOWER_simple_caps",LOWER_CAPS_LENGTH,LOWER_CAPS_RADIUS,fr_lower_leg, fr_kfe, r_lower_offset, rmodel, gmodel, gui=gv)
hl_upper_caps = addCapsule("HL_UPPER_simple_caps",UPPER_CAPS_LENGTH,UPPER_CAPS_RADIUS,hl_upper_leg, hl_hfe, l_upper_offset, rmodel, gmodel, gui=gv)
hl_lower_caps = addCapsule("HL_LOWER_simple_caps",LOWER_CAPS_LENGTH,LOWER_CAPS_RADIUS,hl_lower_leg, hl_kfe, l_lower_offset, rmodel, gmodel, gui=gv)
hr_upper_caps = addCapsule("HR_UPPER_simple_caps",UPPER_CAPS_LENGTH,UPPER_CAPS_RADIUS,hr_upper_leg, hr_hfe, r_upper_offset, rmodel, gmodel, gui=gv)
hr_lower_caps = addCapsule("HR_LOWER_simple_caps",LOWER_CAPS_LENGTH,LOWER_CAPS_RADIUS,hr_lower_leg, hr_kfe, r_lower_offset, rmodel, gmodel, gui=gv)

#gmodel.addCollisionPair(pio.CollisionPair(fl_upper_caps,hl_upper_caps))
gmodel.addCollisionPair(pio.CollisionPair(base_link_geom,fl_lower_leg_geom))
gmodel.addCollisionPair(pio.CollisionPair(base_link_geom,fl_upper_leg_geom))

robot_config[6] = 0.
robot_config[7] = 0.
robot_config[8] = 0
robot_config[9] = 0.
robot_config[10] = 0
robot_config[11] = 0.
robot_config[12] = 0
robot_config[13] = 0.
robot_config[14] = 0
robot_config[15] = 0.
robot_config[16] = 0
robot_config[17] = 0.
robot_config[18] = 0. 

ncol_sim = 0
ncol_ualone = 0
ncol_lalone = 0

faulty_conf = []

for i in range(100):
    # Display the robot
    #robot, rmodel, rdata, gmodel, gdata = initSolo()
    robot.rebuildData() 
    robot.displayCollisions(True)
    robot.displayVisuals(False)
    robot.display(robot_config) 
    for n in gv.getNodeList():
            if 'collision' in n and 'simple' not in n and len(n)>27:
                    gv.setVisibility(n,'ON')

            if 'base_link' in n and 'simple' not in n and len(n)>27:
                    gv.setVisibility(n,'ON')
                    gv.setColor(n, [1,1,1,0.2])

    robot_config[6:18] = -np.pi + 2*np.pi*np.random.random((1,12))
    #robot_config = robot.q0

    gdata  = gmodel.createData()
    pio.computeDistances(rmodel,rdata,gmodel,gdata, robot_config)
    collisions_dist = gdata.distanceResults

    distLL_base = collisions_dist[0].min_distance
    distUL_base = collisions_dist[1].min_distance
    if (distLL_base<=0 and distUL_base<=0):
        ncol_sim +=1
    if (distLL_base<=0 and not distUL_base<=0):
        ncol_lalone +=1
        faulty_conf.append(robot_config.copy())
    if ((not distLL_base<=0 ) and distUL_base<=0):
        ncol_ualone +=1
    p1 = collisions_dist[0].getNearestPoint1() 
    p2 = collisions_dist[0].getNearestPoint2()
    visualizeCollisionDist(p1,p2, str(100*np.random.random()),[0,1,0,1])

    #print(rmodel.nq)
    #print(collisions_dist[0].min_distance, collisions_dist[1].min_distance)

    
    gv.refresh()
print("Simult. {}".format(ncol_sim))
print("UP {}".format(ncol_ualone))
print("LOW {}".format(ncol_lalone))