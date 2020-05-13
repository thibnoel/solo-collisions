import pinocchio as pio
from example_robot_data import loadSolo
import hppfcl
import numpy as np
import matplotlib.pyplot as plt
import time
from numpy.linalg import norm
from solo12_collisions_utils import initSolo, testGeomConfig, computeCollisionMap, followBoundary

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
                        gv.setColor(n, [1,1,1,0.2])
                if 'collision' in n and 'base_link' in n and len(n)>27:
                        gv.setColor(n, [1,0.5,0.2,1])
                if 'collision' in n and 'FL_UPPER_LEG' in n and len(n)>27:
                        gv.setColor(n, [1,0.5,0.2,1])
                        gv.setVisibility(n,'OFF')
        gv.refresh()
'''
# Get robot model, data, and collision model
rmodel = robot.model
rdata  = rmodel.createData()
gmodel = robot.collision_model
#print("Geometric model : \n" + str(gmodel))
#print("Robot model : \n" + str(rmodel))

# Define the robot configuration


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

# Get the base_link and FL_UPPER_LEG meshes
base_link_geom = gmodel.getGeometryId("base_link_0")
fl_upper_geom = gmodel.getGeometryId("FL_UPPER_LEG_0")
# Add the collision pair to the geometric model
gmodel.addCollisionPair(pio.CollisionPair(base_link_geom, fl_upper_geom))

if(enableGUI):
        
gdata = gmodel.createData()
'''

def updateShoulderConfig(x,y):
        robot_config[7] = x
        robot_config[8] = y
        robot.display(robot_config)
        gv.refresh()
        time.sleep(3/(x_discrete_steps))

def visualizeCollisionDist(p1, p2, name, color):
        gv.addSphere("world/" + name + "/p1", .01, color)
        gv.addSphere("world/" + name + "/p2", .01, color)
        gv.applyConfiguration("world/" + name + "/p1", p1.tolist()+[1,0,0,0])
        gv.applyConfiguration("world/" + name + "/p2", p2.tolist()+[1,0,0,0])
        
        gv.refresh()

        ### --- display witness as normal patch tangent to capsule
        for i in range(2):
                gv.addCylinder('world/pinocchio/collisions/simple_patch_' + name + '_%d'%i, .01, .003, color)

        direc = (p2-p1)/norm(p2-p1) 
        capsule_radius = 0.0
        M1 = pio.SE3(pio.Quaternion.FromTwoVectors(np.matrix([0,0,1]).T,p1-p2).matrix(),p1+direc*capsule_radius)
        M2 = pio.SE3(pio.Quaternion.FromTwoVectors(np.matrix([0,0,1]).T,p2-p1).matrix(),p2-direc*capsule_radius)
        gv.applyConfiguration('world/pinocchio/collisions/simple_patch_' + name + '_0',pio.SE3ToXYZQUATtuple(M1))
        gv.applyConfiguration('world/pinocchio/collisions/simple_patch_' + name + '_1',pio.SE3ToXYZQUATtuple(M2))

'''
def relativePlacement(q_robot, rmodel, frameIndex1, frameIndex2):
        robot_config = q_robot
        oMf1 = rmodel.frames[frameIndex1].placement.toHomogeneousMatrix_impl()
        print(oMf1)
        oMf2 = rmodel.frames[frameIndex2].placement
        print(oMf2)
        f1Mf2 = np.linalg.inv(oMf1).dot(oMf2)
        return f1Mf2
'''
                
x_rot_range = [-np.pi, np.pi]
y_rot_range = [-np.pi, np.pi]
x_discrete_steps = 100
y_discrete_steps = 100

col_map, nearest_points_map = computeCollisionMap(robot_config, x_rot_range, y_rot_range, x_discrete_steps, y_discrete_steps, rmodel, rdata, gmodel, gdata)
col_map = np.array(col_map)
binary_col_map = np.array(col_map) > 0

#np.save('./collision_map_centered_res1000', col_map)

traj = followBoundary(col_map, first_dir=0)
#traj002 = followBoundary(col_map, dist_threshold=0.002)
#traj001 = followBoundary(col_map, dist_threshold=0.001)

# Animate the boundary trajectory in the viewer
def displayTraj(traj):
        for t in traj:
                updateShoulderConfig(t[1]*2*np.pi/x_discrete_steps, t[0]*2*np.pi/x_discrete_steps)
                visualizeCollisionDist(nearest_points_map[t[0]][t[1]][0], nearest_points_map[t[0]][t[1]][1], "test", [0.2,1,0,0.5])

#plt.figure()
plt.matshow(binary_col_map.astype(float), extent=[x_rot_range[0], x_rot_range[1], y_rot_range[0],y_rot_range[1]])
plt.xlabel('Shoulder X rot.')
plt.ylabel('Shoulder Y rot.')

#plt.colorbar(label='Dist. Body to upper leg')

#plt.figure()
plt.matshow(np.array(col_map), extent=[x_rot_range[0], x_rot_range[1], y_rot_range[0],y_rot_range[1]])#, vmin=0)
plt.colorbar(label='Dist. Body to upper leg')
plt.xlabel('Shoulder X rot.')
plt.ylabel('Shoulder Y rot.')
plt.scatter([x_rot_range[0] + t[0]*(x_rot_range[1] - x_rot_range[0])/x_discrete_steps for t in traj], 
            [y_rot_range[1] - t[1]*(y_rot_range[1] - y_rot_range[0])/y_discrete_steps for t in traj],
            color='r', s=10)
#plt.scatter([t[0] for t in traj001], [t[1] for t in traj001], color='r', s=5)
#plt.scatter([t[0] for t in traj002], [t[1] for t in traj002], color='r', s=2)
plt.show()

#print(relativePlacement(robot_config, rmodel, fl_upper_leg, base_link))
