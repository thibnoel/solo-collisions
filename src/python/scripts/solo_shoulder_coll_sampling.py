from example_robot_data import loadSolo
from collision_sampling.articular_space_coll_sampling import *
from collision_sampling.articular_space_coll_visualization import *


# Initialize SOLO12 model with the 2 collisions pairs of the FL shoulder
def initSolo():
    robot = loadSolo(solo=False)
    # Get robot model, data, and collision model
    rmodel = robot.model
    rdata  = rmodel.createData()
    gmodel = robot.collision_model

    # Get the base_link and FL_UPPER_LEG geometries
    base_link_geom = gmodel.getGeometryId("base_link_0")
    upper_leg_geom = gmodel.getGeometryId("FL_UPPER_LEG_0")
    lower_leg_geom = gmodel.getGeometryId("FL_LOWER_LEG_0")
    # Add the collision pairs to the geometric model
    gmodel.addCollisionPair(pio.CollisionPair(base_link_geom, upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(base_link_geom, lower_leg_geom))

    gdata = gmodel.createData()
    return robot, rmodel, rdata, gmodel, gdata


##### MAIN #####
if __name__ == "__main__":
    # execute only if run as a script
    robot, rmodel, rdata, gmodel, gdata = initSolo()
    ref_config = np.zeros(robot.q0.shape)

    # Initiliaze joints ranges to sample
    x_rot_range = [-np.pi, np.pi]
    y_rot_range = [-np.pi, np.pi]
    knee_rot_range = [-np.pi, np.pi]    

    # Choose 2D (shoulder DoF only) or 3D (shoulder + knee DoF) sampling of the collision in articular space
    coll_3d = False
    
    if coll_3d:
        # Initialize sampling parameters
        # 3D : shoulder + knee DoF
        q_ind = [7,8,9]
        q_ranges = [x_rot_range, y_rot_range, knee_rot_range]
        q_steps = [60,60,9]

        collisionPairs = [0,1]

        # Sample FCL distance
        grid_config = generateGridConfigs(q_ind, q_ranges, q_steps)
        #rand_config = generateRandomConfigs(5000, q_ind, q_ranges)
        col_map = sampleFCLDistanceFromConfigList(ref_config, q_ind, grid_config, collisionPairs, rmodel, rdata, gmodel, gdata, computeDist=False)

        # Load an existing sampled boundary
        bound3d = np.load("/home/tnoel/npy_data/npy_data/npy_data/3d_bound_ref_200x200.npy", allow_pickle=True)
        bound = bound3d
    
    else:
        # Initialize sampling parameters
        # 2D : shoulder DoF
        q_ind = [7,8]
        q_ranges = [x_rot_range, y_rot_range]
        q_steps = [200,200]

        collisionPairs = [0]
        
        # Sample FCL distance
        grid_config = generateGridConfigs(q_ind, q_ranges, q_steps)
        #rand_config = generateRandomConfigs(5000, q_ind, q_ranges)
        col_map = sampleFCLDistanceFromConfigList(ref_config, q_ind, grid_config, collisionPairs, rmodel, rdata, gmodel, gdata, computeDist=False)
        #col_map_d = sampleFCLDistanceFromConfigList(ref_config, q_ind, grid_config, collisionPairs, rmodel, rdata, gmodel, gdata, computeDist=True)
    
        # Example of boundary estimation between 2 known configuration planes
        '''
        bound_ref_dir = 0
        bound_ref_val0 = [0,-np.pi]
        bound_ref_val1 = [0, np.pi]
        
        bound0 = boundaryPlaneSampling(q_ind, bound_ref_dir, bound_ref_val0, q_ranges[1:], 100, 1e-9, ref_config, collisionPairs, rmodel, rdata, gmodel, gdata, extend_periodic=False)
        bound1 = boundaryPlaneSampling(q_ind, bound_ref_dir, bound_ref_val1, q_ranges[1:], 100, 1e-9, ref_config, collisionPairs, rmodel, rdata, gmodel, gdata, extend_periodic=False)
        bound = np.concatenate((bound0, bound1))
        '''
        # Example of randomly sampled boundary estimation
        plt.figure()
        bound = boundaryRandomSapling(q_ind, q_ranges, 20, 1e-9, ref_config, [0,1], rmodel, rdata, gmodel, gdata, extend_periodic=True)
        # Load an existing sampled boundary
        #bound2d = np.load("/home/tnoel/npy_data/npy_data/npy_data/2d_bound_ref_5000samp.npy", allow_pickle=True)
        #bound = bound2d
    

    # Convert spatial distance to articular distance
    # Compute jacobian as well
    dist_metric = 'euclidean'
    articular_col_map = spatialToArticular(col_map, bound, batch_size=100, metric=dist_metric)
    articular_jacobian = getSampledJac(articular_col_map, bound, batch_size=100, metric=dist_metric)
    #init_jacobian = getSampledJac(col_map_d, bound, batch_size=100, metric=dist_metric)

    #articular_jacobian = init_jacobian

    ######## Results visualization
    # 3D
    if coll_3d:    
        # 3D visualization    
        FCL_viz = col_map 
        FCL_viz[:,-1] = np.sign(FCL_viz[:,-1])*FCL_viz[:,2]
        fig = plt.figure()
        visualize3DData(fig, FCL_viz[np.where(FCL_viz[:,-1] > 0)], q_ind, q_ranges, grid=False, q_steps=q_steps, subplot_pos=211, title="FCL Dist. > 0", cmap=plt.cm.viridis)
        visualize3DData(fig, FCL_viz[np.where(FCL_viz[:,-1] <= 0)], q_ind, q_ranges, grid=False, q_steps=q_steps, subplot_pos=212, title="FCL Dist. <= 0", cmap=plt.cm.viridis)
        
        # 2D-flattened visualization
        plt.figure()
        visualizeFlat3DData(articular_col_map, q_ind, q_ranges, q_steps, title="Articular distance", cmap=plt.cm.RdYlGn)

        jac_x = articular_jacobian[:,[0,1,2,3]]
        jac_y = articular_jacobian[:,[0,1,2,4]]
        jac_z = articular_jacobian[:,[0,1,2,5]]

        # Jacobians visualization
        fig2 = plt.figure()
        visualize3DData(fig2, jac_x, q_ind, q_ranges, grid=False, q_steps=q_steps, subplot_pos=131, title="J_q7", cmap=plt.cm.PiYG)
        visualize3DData(fig2, jac_y, q_ind, q_ranges, grid=False, q_steps=q_steps, subplot_pos=132, title="J_q8", cmap=plt.cm.PiYG)
        visualize3DData(fig2, jac_z, q_ind, q_ranges, grid=False, q_steps=q_steps, subplot_pos=133, title="J_q9", cmap=plt.cm.PiYG)
    
    # 2D
    else:
        # Visualize distance results
        plt.figure()
        #plt.subplot(1,3,1)
        #visualize2DData(col_map_d, q_ind, q_ranges, grid=True, q_steps=q_steps, title="FCL distance", cmap=plt.cm.viridis)
        #plt.colorbar()
        plt.subplot(1,3,2)
        visualize2DData(col_map, q_ind, q_ranges, grid=True, q_steps=q_steps, title="Binary collision", cmap=plt.cm.gray)
        plt.colorbar()
        plt.subplot(1,3,3)
        visualize2DData(articular_col_map, q_ind, q_ranges, grid=True, q_steps=q_steps, title="Articular distance", cmap=plt.cm.RdYlGn)
        plt.colorbar()

        # visualize jacobians results
        plt.figure()
        plt.subplot(2,1,1)
        visualize2DData(articular_jacobian[:,[0,1,2]], q_ind, q_ranges, grid=True, q_steps=q_steps, title="J_q7", cmap=plt.cm.PiYG)
        plt.subplot(2,1,2)
        visualize2DData(articular_jacobian[:,[0,1,3]], q_ind, q_ranges, grid=True, q_steps=q_steps, title="J_q8", cmap=plt.cm.PiYG)

    plt.show()