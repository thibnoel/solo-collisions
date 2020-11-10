from example_robot_data import loadTalos
from collision_sampling.articular_space_coll_sampling import *
from collision_sampling.articular_space_coll_visualization import *

def initTalos():
    robot = loadTalos()
    # Get robot model, data, and collision model
    rmodel = robot.model
    rdata  = rmodel.createData()
    gmodel = robot.collision_model

    geom_names = ["torso_2_link_0",
                    #"arm_left_1_link_0",
                    "arm_left_2_link_0",
                    "arm_left_3_link_0",
                    "arm_left_4_link_0"]
                    #"arm_left_5_link_0"]
                    #"arm_left_6_link_0",
                    #"arm_left_7_link_0"]
    geom_IDs = [gmodel.getGeometryId(name) for name in geom_names]

    n_geom = len(geom_IDs)
    npairs = 0
    '''
    for i in range(n_geom):
        for j in range(i+1, n_geom):
            p = pio.CollisionPair(geom_IDs[i], geom_IDs[j])
            gmodel.addCollisionPair(p)
    '''
    gmodel.addCollisionPair(pio.CollisionPair(geom_IDs[0], geom_IDs[1]))
    gmodel.addCollisionPair(pio.CollisionPair(geom_IDs[0], geom_IDs[2]))
    gmodel.addCollisionPair(pio.CollisionPair(geom_IDs[0], geom_IDs[3]))
    #gmodel.addCollisionPair(pio.CollisionPair(geom_IDs[0], geom_IDs[4]))
    #gmodel.addCollisionPair(pio.CollisionPair(geom_IDs[0], geom_IDs[5]))

    npairs = len(gmodel.collisionPairs)

    gdata = gmodel.createData()
    return robot, rmodel, rdata, gmodel, gdata, geom_names, npairs


##### MAIN #####
if __name__ == "__main__":
    robot, rmodel, rdata, gmodel, gdata, geom_names, npairs = initTalos()
    robot_config = robot.q0
    robot_config[23] += 0.8

    enableGUI = False

    if(enableGUI):
        robot.initViewer(loadModel=True)
        gv = robot.viewer.gui
        # Display the robot
        robot.rebuildData() 
        robot.displayCollisions(True)
        robot.displayVisuals(False)
        robot.display(robot_config) 
        for n in gv.getNodeList():
            print(n)
            if 'collision' in n and len(n)>27:
                gv.setVisibility(n,'ON')
                gv.setColor(n, [1,1,1,1])
            for g in geom_names:
                if(g in n):
                    #gv.setVisibility(n,'OFF')
                    gv.setColor(n, [1,0.5,0,1])
        gv.refresh()

    # Initialize joints ranges to sample
    q0_range = [0 - 2.4, 2*np.pi - 2.4]
    q1_range = [0 - 2.4, 2*np.pi - 2.4]
    q2_range = [0, 2*np.pi]

    # Initialize sampling parameters
    q_ind = [21,22,24]
    q_ranges = [q0_range, q1_range, q2_range]
    q_steps = [75,75,22]

    # Sample FCL distance
    grid_config = generateGridConfigs(q_ind, q_ranges, q_steps)
    col_map = sampleFCLDistanceFromConfigList(robot_config, q_ind, grid_config, [k for k in range(npairs)], rmodel, rdata, gmodel, gdata, computeDist=False)

    # Sample collision boundary
    bound = boundaryRandomSapling(q_ind, q_ranges, 5000, 1e-6, robot_config, [k for k in range(npairs)], rmodel, rdata, gmodel, gdata, extend_periodic=True)

    # Convert distance to articular distance
    dist_metric='euclidean'
    articular_col_map = spatialToArticular(col_map, bound, batch_size=100, metric=dist_metric)
    #articular_jacobian = getSampledJac(articular_col_map, bound, batch_size=100, metric=dist_metric)

    #col_map = articular_col_map

    ######## Results visualization
    # 3D visualization
    fig = plt.figure()
    visualize3DData(fig, articular_col_map[np.where(col_map[:,-1] > 0)], q_ind, q_ranges, grid=False, q_steps=q_steps, subplot_pos=211, title="Dist. > 0", cmap=plt.cm.viridis, vmin=np.min(col_map[:,-1]))
    visualize3DData(fig, articular_col_map[np.where(col_map[:,-1] <= 0)], q_ind, q_ranges, grid=False, q_steps=q_steps, subplot_pos=212, title="Dist. <= 0", cmap=plt.cm.viridis, vmin=np.min(col_map[:,-1]))
    
    # 2D flattened visualization
    plt.figure()
    visualizeFlat3DData(articular_col_map, q_ind, q_ranges, q_steps, title="FCL distance", cmap=plt.cm.RdYlGn)
'''
    jac0 = articular_jacobian[:,[0,1,2,3]]
    jac1 = articular_jacobian[:,[0,1,2,4]]
    jac2 = articular_jacobian[:,[0,1,2,5]]

    # Jacobians visualization
    fig2 = plt.figure()
    visualize3DData(fig2, jac0, q_ind, q_ranges, grid=False, q_steps=q_steps, subplot_pos=131, title="J_q0", cmap=plt.cm.PiYG)
    visualize3DData(fig2, jac1, q_ind, q_ranges, grid=False, q_steps=q_steps, subplot_pos=132, title="J_q1", cmap=plt.cm.PiYG)
    visualize3DData(fig2, jac2, q_ind, q_ranges, grid=False, q_steps=q_steps, subplot_pos=133, title="J_q2", cmap=plt.cm.PiYG)
'''
    #plt.show()