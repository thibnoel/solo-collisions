import pinocchio as pio
from example_robot_data import loadSolo
from example_robot_data.robots_loader import readParamsFromSrdf, addFreeFlyerJointLimits, getModelPath
import hppfcl
import numpy as np
import matplotlib.pyplot as plt
import time
from numpy.linalg import norm

# Write a load function to adapt until Solo12 is in example-robot-data
def TEMP_loadSolo12():
        URDF_FILENAME = "solo12.urdf"
        SRDF_FILENAME = "solo.srdf"
        SRDF_SUBPATH = "/solo_description/srdf/" + SRDF_FILENAME
        URDF_SUBPATH = "/solo_description/robots/" + URDF_FILENAME
        modelPath = "/home/thibault/stage_LAAS_042020_102020"
        #modelPath = getModelPath(URDF_SUBPATH)
        # Load URDF file
        robot = pio.RobotWrapper.BuildFromURDF(modelPath + URDF_SUBPATH, ["/home/thibault/stage_LAAS_042020_102020"], root_joint=pio.JointModelFreeFlyer())
        # Load SRDF file
        readParamsFromSrdf(robot, modelPath + SRDF_SUBPATH, False, False, "standing")
        # Add the free-flyer joint limits
        addFreeFlyerJointLimits(robot)
        return robot

#robot = loadSolo(solo=False) # solo=True : solo 8, solo=False : solo 12
robot = TEMP_loadSolo12()

# Initialize the gepetto-gui viewer
enableGUI = True

if(enableGUI):
        robot.initViewer(loadModel=True)
        gv = robot.viewer.gui

# Get robot model, data, and collision model
rmodel = robot.model
rdata  = rmodel.createData()
gmodel = robot.collision_model
#print("Geometric model : \n" + str(gmodel))
#print("Robot model : \n" + str(rmodel))

# Define the robot configuration
robot_config = np.zeros(robot.q0.shape) # Default configuration

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
gdata = gmodel.createData()

def testGeomConfig(x_rot, y_rot, collisionPair):
        test_config = robot_config.copy()

        test_config[7] = x_rot
        test_config[8] = y_rot 

        pio.computeDistances(rmodel,rdata,gmodel,gdata, test_config)
        pio.computeCollisions(rmodel, rdata, gmodel, gdata, test_config, True)

        # Get distance and collision results from collision data
        collisions_dist = gdata.distanceResults
        collisions = gdata.collisionResults 
        is_col = collisions[collisionPair].isCollision() # pair 0 is between the leg and the body
        dist = collisions_dist[collisionPair].min_distance

        p1 = collisions_dist[collisionPair].getNearestPoint1() 
        p2 = collisions_dist[collisionPair].getNearestPoint2() # get nearest points on the segments

        return is_col, dist, [p1,p2]

def computeCollisionMap(x_rot_range, y_rot_range, xsteps, ysteps):
        col_map = []
        nearest_points_map = []

        for i in range(y_discrete_steps):
                i_col_map = []
                i_nearest_points_map = []
                #robot_config[7] += (y_rot_range[1] - y_rot_range[0])/y_discrete_steps
                for j in range(x_discrete_steps):
                        colTest = testGeomConfig(x_rot_range[0] + j*(x_rot_range[1] - x_rot_range[0])/x_discrete_steps, y_rot_range[0] + i*(y_rot_range[1] - y_rot_range[0])/y_discrete_steps, 0)
                        i_col_map.append(colTest[1])
                        i_nearest_points_map.append(colTest[2])
                col_map.append(i_col_map)
                nearest_points_map.append(i_nearest_points_map)
        return col_map, nearest_points_map

def updateConfig(x,y):
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

def followBoundary(col_map, dist_threshold=0, first_dir=6):
        curr_x = 0
        curr_y = 0
        dirList = [[1,0],
                   [1,1],
                   [0,1],
                   [-1,1],
                   [-1,0],
                   [-1,-1],
                   [0,-1],
                   [1,-1]]

        curr_dir_index = first_dir
        first_dir_change_index = 0
        traj = [[0,0]]
        # Stopping condition : no more move possible or reached x = max or y = max
        inCollision = False
        counter = 0

        mapSize = len(col_map)
        while(curr_x < mapSize and curr_y < mapSize):
                next_front_x = (curr_x + dirList[curr_dir_index][0] + mapSize)%mapSize
                next_front_y = (curr_y + dirList[curr_dir_index][1] + mapSize)%mapSize

                #print(next_front_x, next_front_y)
                next_left_x = (curr_x + dirList[curr_dir_index - 2][0] + mapSize)%mapSize
                next_left_y = (curr_y + dirList[curr_dir_index - 2][1] + mapSize)%mapSize

                nextFrontColl = col_map[next_front_y,next_front_x]
                if(nextFrontColl <= dist_threshold):
                        inCollision = True
                else:
                        inCollision = False

                if(inCollision):
                        curr_dir_index += 1
                        curr_dir_index %= 8
                        if (first_dir_change_index == 0):
                                first_dir_change_index = counter
                
                else:
                        curr_x = next_front_x
                        curr_y = next_front_y
                
                if([curr_x, curr_y] != traj[-1]):       
                        traj.append([curr_x, curr_y])

                nextLeftColl = col_map[next_left_y, next_left_x]
                if(nextLeftColl <= dist_threshold and not inCollision):
                        curr_dir_index -=1
                        curr_dir_index = (8 + curr_dir_index)%8

                if(counter > mapSize*3):
                        break
                counter += 1
        #print(traj)
        return traj[first_dir_change_index:]
                        
x_rot_range = [-np.pi, np.pi]
y_rot_range = [0, 2*np.pi]
x_discrete_steps = 200
y_discrete_steps = 200

col_map, nearest_points_map = computeCollisionMap(x_rot_range, y_rot_range, x_discrete_steps, y_discrete_steps)
col_map = np.array(col_map).T
binary_col_map = np.array(col_map) > 0

#traj = followBoundary(col_map)
#traj002 = followBoundary(col_map, dist_threshold=0.002)
#traj001 = followBoundary(col_map, dist_threshold=0.001)

# Animate the boundary trajectory in the viewer
def displayTraj(traj):
        for t in traj:
                updateConfig(t[1]*2*np.pi/x_discrete_steps, t[0]*2*np.pi/x_discrete_steps)
                visualizeCollisionDist(nearest_points_map[t[0]][t[1]][0], nearest_points_map[t[0]][t[1]][1], "test", [0.2,1,0,0.5])

#plt.figure()
plt.matshow(binary_col_map.astype(float), extent=[y_rot_range[0], y_rot_range[1], x_rot_range[0],x_rot_range[1]])
plt.xlabel('Shoulder Y rot.')
plt.ylabel('Shoulder X rot.')

#plt.colorbar(label='Dist. Body to upper leg')

#plt.figure()
plt.matshow(np.array(col_map), extent=[x_rot_range[0], x_rot_range[1], y_rot_range[0],y_rot_range[1]])#, vmin=0)
plt.colorbar(label='Dist. Body to upper leg')
plt.xlabel('Shoulder Y rot.')
plt.ylabel('Shoulder X rot.')
#plt.scatter([t[0] for t in traj], [t[1] for t in traj], color='r', s=10)
#plt.scatter([t[0] for t in traj001], [t[1] for t in traj001], color='r', s=5)
#plt.scatter([t[0] for t in traj002], [t[1] for t in traj002], color='r', s=2)

plt.show()