import pinocchio as pio
from example_robot_data import loadSolo
import hppfcl
import numpy as np
import matplotlib.pyplot as plt
import time
from numpy.linalg import norm

def initSolo():
    robot = loadSolo(solo=False)
    # Get robot model, data, and collision model
    rmodel = robot.model
    rdata  = rmodel.createData()
    gmodel = robot.collision_model

    # Get the base_link and FL_UPPER_LEG meshes
    base_link_geom = gmodel.getGeometryId("base_link_0")
    fl_upper_geom = gmodel.getGeometryId("FL_UPPER_LEG_0")
    # Add the collision pair to the geometric model
    gmodel.addCollisionPair(pio.CollisionPair(base_link_geom, fl_upper_geom))

    gdata = gmodel.createData()
    return robot, rmodel, rdata, gmodel, gdata

def testGeomConfig(config, x_rot, y_rot, collisionPair, rmodel, rdata, gmodel, gdata):
        test_config = config.copy()

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

def computeCollisionMap(config, x_rot_range, y_rot_range, xsteps, ysteps, rmodel, rdata, gmodel, gdata):
        col_map = []
        nearest_points_map = []

        for i in range(ysteps):
                i_col_map = []
                i_nearest_points_map = []
                for j in range(xsteps):
                        colTest = testGeomConfig(config, x_rot_range[0] + j*(x_rot_range[1] - x_rot_range[0])/xsteps, y_rot_range[0] + i*(y_rot_range[1] - y_rot_range[0])/ysteps, 0, rmodel, rdata, gmodel, gdata)
                        i_col_map.append(colTest[1])
                        i_nearest_points_map.append(colTest[2])
                col_map.append(i_col_map)
                nearest_points_map.append(i_nearest_points_map)
        return col_map, nearest_points_map

def followBoundary(col_map, dist_threshold=0, first_dir=6):
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
        # Stopping condition : no more move possible or reached x = max or y = max
        inCollision = False
        counter = 0

        mapYSize = len(col_map)
        mapXSize = len(col_map[0])

        curr_x = 0
        curr_y = int(mapYSize/2)
        traj = [[curr_x,curr_y]]

        while(curr_x < mapXSize and curr_y < mapYSize):
                next_front_x = (curr_x + dirList[curr_dir_index][0] + mapXSize)%mapXSize
                next_front_y = (curr_y + dirList[curr_dir_index][1] + mapYSize)%mapYSize

                #print(next_front_x, next_front_y)
                next_left_x = (curr_x + dirList[curr_dir_index - 2][0] + mapXSize)%mapXSize
                next_left_y = (curr_y + dirList[curr_dir_index - 2][1] + mapYSize)%mapYSize

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
                        if([curr_x, curr_y] in traj):
                                break    
                        traj.append([curr_x, curr_y])

                nextLeftColl = col_map[next_left_y, next_left_x]
                if(nextLeftColl <= dist_threshold and not inCollision):
                        curr_dir_index -=1
                        curr_dir_index = (8 + curr_dir_index)%8

                if(counter > max(mapXSize, mapYSize)*3):
                        break
                counter += 1
        #print(traj)
        return traj[first_dir_change_index:]