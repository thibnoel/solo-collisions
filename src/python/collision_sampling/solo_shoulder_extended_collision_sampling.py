import pinocchio as pio
from example_robot_data import loadSolo
import hppfcl
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from scipy.spatial import distance
from numpy.linalg import norm


# Initialize SOLO model
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


# Evaluates the collision distance (mesh-to-mesh) with FCL for a given configuration
# If multiple pairs are given, returns the min. of them all
def sampleFCLDistanceFromConfig(ref_config, q_ind, q_val, collisionPairs, rmodel, rdata, gmodel, gdata, computeDist=True):
    test_config = ref_config.copy() # to change; config can be any valid config

    for k in range(len(q_ind)):
        test_config[q_ind[k]] = q_val[k]

    if(computeDist):
        pio.computeDistances(rmodel, rdata, gmodel, gdata, test_config)
    else:
        pio.computeCollisions(rmodel, rdata, gmodel, gdata, test_config, True)
    # Get distance and collision results from collision data
    if(computeDist):
        collisions_dist = gdata.distanceResults
    else:
        collisions_dist = gdata.collisionResults
    distances = []
    for c in collisionPairs:
        if(computeDist):
            distances.append(collisions_dist[c].min_distance)  # pair 0 is between the FL upper leg and the body, see initSolo(), 1 between lower leg and body
        else:
            distances.append(float(not collisions_dist[c].isCollision()))
    dist = np.min(distances)
    # Get nearest points on the meshes - FOR REF.
    #p1 = collisions_dist[collisionPair].getNearestPoint1() 
    #p2 = collisions_dist[collisionPair].getNearestPoint2() 

    return dist


# Sample FCL spatial distance on grid-aligned points
# Arguments :
#   - q_nb_steps : nb. of grid divisions in each direction
def sampleGridCollisionMap(config, q_ind, q_ranges, q_nb_steps, collisionPairs, rmodel, rdata, gmodel, gdata, computeDist=True):

    def qFromInd(index):
        q = []
        for k in range(len(index)):
            q.append(q_ranges[k][0] + index[k]*(q_ranges[k][1] - q_ranges[k][0])/q_nb_steps[k])
        return q

    def updateGridInd(curr_index, k):
        if(k>=len(curr_index)):
            return curr_index
        if(curr_index[k] < q_nb_steps[k] - 1):
            curr_index[k] += 1
            return curr_index
        else:
            curr_index[k] = 0
            curr_index = updateGridInd(curr_index, k+1)
            return curr_index
                
    col_map = []
    counter = 0
    ind = [0]*len(q_ranges)
    nb_samples = np.array([q_nb_steps]).prod()
    while(counter < nb_samples):
        q = qFromInd(ind)
        d = sampleFCLDistanceFromConfig(config, q_ind, q, collisionPairs, rmodel, rdata, gmodel, gdata, computeDist=computeDist)

        result = []
        result.extend(q)
        result.append(d)
        col_map.append(result)
        
        # Update current grid index
        ind = updateGridInd(ind,0)
        counter += 1

        if(counter%100 == 0):
            print("<GRID DIST. SAMPLING> Generated {} / {} dist. samples".format(counter, nb_samples), end='\r')
    print('')

    col_map = np.array(col_map)
    return col_map


# Samples the FCL spatial distance for random configurations
def sampleRandomCollisionMap(config, q_ind, q_ranges, npoints, collisionPairs, rmodel, rdata, gmodel, gdata, computeDist=True):
    col_map = np.zeros((npoints, len(q_ind)+1))
    k = 0
    for k in range(npoints):
        rand_q = []
        for i in range(len(q_ind)):
            rand_q.append(q_ranges[i][0] + np.random.rand()*(q_ranges[i][1] - q_ranges[i][0]))
        dist = sampleFCLDistanceFromConfig(config, q_ind, rand_q, collisionPairs, rmodel, rdata, gmodel, gdata, computeDist=computeDist)

        for i in range(len(q_ind)):
            col_map[k,i] = rand_q[i]
        col_map[k,len(q_ind)] = dist
        if( (k+1) % 100 == 0):
            print("<RANDOM DIST. SAMPLING> Generated {} / {} dist. samples".format(k+1, npoints), end='\r')
    print('')
    return col_map


# Returns a boundary configuration lying outside the collision zone
# Arguments :
#   - q_free, q_coll : reference configurations in the free zone and collision zone
#       --> q_bound will be returned as q_bound = q_free + t*(q_coll - q_free), t in [0,1]
#   - threshold : max articular distance to the boundary (precision of the boundary approx.)
def dichotomyBoundaryLoc(q_free, q_coll, threshold, robot_config, q_ind, collisionPairs, rmodel, rdata, gmodel, gdata):
    qf = q_free.copy()
    qc = q_coll.copy()
    while((qf-qc).T@(qf-qc) > threshold):
        new_q = 0.5*(qc+qf)
        d = sampleFCLDistanceFromConfig(robot_config, q_ind, new_q, collisionPairs, rmodel, rdata, gmodel, gdata)
        if(d>0):
            qf = new_q
        else:
            qc = new_q
    return qf # ensure the boundary points are always sampled in the free zone 


# Returns the boundary sampled between 2 reference planes aligned with and offset in a direction of q
# 1 should contain only collision configurations, the other one only free configurations
# !!! only works for special cases (depending on boundary shape)
def dichotomyBoundaryPlaneSampling(q_ind, q_ref_dir, q_ref_vals, q_ranges, q_res, threshold, ref_config, collisionPairs, rmodel, rdata, gmodel, gdata, extend_periodic=False):
    def qFromInd(index, qFixedVal):
        q = []
        for k in range(len(index)):
            if (k == q_ref_dir):
                q.append(qFixedVal)
            q.append(q_ranges[k][0] + index[k]*(q_ranges[k][1] - q_ranges[k][0])/q_res)
        return q

    def updateGridInd(curr_index, k):
        if(k>=len(curr_index)):
            return curr_index
        if(curr_index[k] < q_res - 1):
            curr_index[k] += 1
            return curr_index
        else:
            curr_index[k] = 0
            curr_index = updateGridInd(curr_index, k+1)
            return curr_index

    nb_ref_points = np.prod((len(q_ind)-1)*[q_res])
    bound = []
    ind = [0]*(len(q_ind) - 1)

    q_free_ref = q_ref_vals[0]
    q_coll_ref = q_ref_vals[1]

    for i in range(nb_ref_points):
        qf = np.array(qFromInd(ind, q_free_ref))
        qc = np.array(qFromInd(ind, q_coll_ref))
        qs = dichotomyBoundaryLoc(qf, qc, threshold,ref_config, q_ind, collisionPairs, rmodel, rdata, gmodel, gdata)
        bound.append(qs)
        '''
        ax.scatter(qf[0], qf[1], qf[2], c='green',s=80)
        ax.scatter(qc[0], qc[1], qc[2], c='red',s=80)
        ax.plot([qf[0], qc[0]],[qf[1], qc[1]],[qf[2], qc[2]], c='grey')
        '''
        ind = updateGridInd(ind, 0)

        if((i+1) % 10 == 0):
            print("<BOUNDARY SAMPLING> Computed {} / {} boundary config.".format((i+1), nb_ref_points), end='\r')
    print('')

    bound = np.array(bound)

    if(extend_periodic):
        add_bounds = [bound]
        L = len(q_ind)
        for k in range(L):
            offset_ind = np.array((L-k-1)*[0]+[1]+k*[0])
            add_bounds.append(bound + 2*np.pi*offset_ind)
            add_bounds.append(bound - 2*np.pi*offset_ind)
        bound = np.concatenate(add_bounds)
    return bound
    

# Converts a spatial distance (as returned by FCL) to articular distance based on boundary configurations
# Samples : size (n,3), holding [qx, qy, fcl_dist]
def spatialToArticular(dist_samples, bounds, batch_size=100, metric='euclidean'):
    # Evaluate in batches to avoid memory overload
    dim = bounds.shape[1]
    npoints = dist_samples.shape[0]

    samples = dist_samples.copy()
    for k in range(int(npoints/batch_size)):
        loc_samples = samples[k*batch_size:(k+1)*batch_size]
        updated_dist = np.min(distance.cdist(bounds, loc_samples[:,:dim], metric=metric), axis=0)
        updated_dist *= -1*(loc_samples[:,dim]<=0) + (loc_samples[:,dim] > 0)
        loc_samples[:,dim] = updated_dist
        print("<SPATIAL_TO_ARTICULAR> Converted {} / {} dist. samples to articular".format((k+1)*batch_size, npoints), end='\r')
    print('')
    return samples


# Computes the Jacobian of the articular distance as :
#   - q : current config
#   - w : witness collision config
#   - u = q-w
#   --> J = (1/u.T*u)*u = (1/d)*u
def getSampledJac(dist_samples, bounds, batch_size=100, metric='euclidean'):
    # Evaluate in batches to avoid memory overload
    dim = bound.shape[1]
    npoints = dist_samples.shape[0]

    samples = dist_samples.copy()
    full_J = []
    for k in range(int(npoints/batch_size)):
        loc_samples = samples[k*batch_size:(k+1)*batch_size]
        witness_config_ind = np.argmin(distance.cdist(bounds, loc_samples[:,:dim], metric=metric), axis=0)
        witness_config = bounds[witness_config_ind]
        J = (loc_samples[:,:dim] - witness_config)
        for d in range(dim):
            J[:,d] /= loc_samples[:,dim]
        full_J.append(J)
        print("<J> Computed {} / {} jac. samples".format((k+1)*batch_size, npoints), end='\r')
    print('')
    return np.concatenate(full_J)


def getSampledFiniteDiffJac(dist_samples, bounds, dq=0.01):
    dim = bound.shape[1]
    npoints = dist_samples.shape[0]

    Jac = []
    
    for k in range(npoints):
        q = dist_samples[k,:dim]
        dist = dist_samples[k, -1]

        dq = np.zeros(dim)
        for i in range(dim):
            dq[i] = dqi

            

            dq[i] = 0

# Sample articular distance on randomly placed points, but with higher probability around the boudary configurations
# Arguments :
#   - bound : list of boundary configurations to compute the distance against
#   - d_nb_points : nb. of samples

# N-D NOT WORKING
def sampleAroundBoundDistanceMap(config, q_ind, q_ranges, nb_points, bound, collisionPairs, rmodel, rdata, gmodel, gdata, computeDist=True):

    def sampleAround(point, min_dist, max_dist, l, nb_samples):
        points = []
        for k in range(nb_samples):
            val = np.random.rand()
            qdist = min_dist + max_dist*np.exp(-l*val)
            # Dirty random direction
            rand_dir = 1+np.random.random((len(q_ind)))
            rand_dir = np.multiply(rand_dir, 0.5 - np.array(np.random.random(len(q_ind)) > 0.5) )
            rand_dir = rand_dir/(rand_dir.T@rand_dir)
            # Idea : q = qcenter + d*rand_dir
            rand_q = point + rand_dir*qdist

            d = sampleFCLDistanceFromConfig(config, q_ind, rand_q, collisionPairs, rmodel, rdata, gmodel, gdata, computeDist=computeDist)
            result = []
            result.extend(rand_q)
            result.append(d)
            points.append(result)
        return np.array(points)    

    # Get the right index for a bound saved with extensions for periodicity
    def randomIndex():
        L = bound.shape[0]
        rand_side =  np.random.randint(0,2)
        rand_ind = np.random.randint(0, int( L/ (2*(2*len(q_ind) + 1)) ) )
        return int(L/2)*rand_side + rand_ind
    
    count = 0
    points = []
    J = []
    while(count < nb_points):
        per_point = 10
        rand_ind = randomIndex()
        p = bound[rand_ind]
        new_samples = sampleAround(p, 1e-3, 0.5*np.pi, 5, per_point)
        points.append(new_samples)
        count += per_point
        print("<AROUND BOUND DIST. SAMPLING> Generated {} / {} dist. samples".format(count, nb_points), end='\r')
    print('')
    points = np.concatenate(points)
    
    return points


def plot2DGridDistance(col_map, q_ind, q_ranges, q_steps):
    #2D
    grid_data = col_map[:,2].reshape(q_steps[1], q_steps[0])
    
    plt.figure()
    plt.subplot(2,1,1)
    plt.imshow(grid_data, extent=q_ranges[0]+q_ranges[1], cmap=plt.cm.RdYlGn, vmin=np.min(col_map[:,2]))
    plt.xlabel("q[{}] - {} ticks".format(q_ind[0], q_steps[0]))
    plt.ylabel("q[{}] - {} ticks".format(q_ind[1], q_steps[1]))
    plt.title("Distance")

    plt.subplot(2,1,2)
    plt.imshow(grid_data > 0, extent=q_ranges[0]+q_ranges[1])
    plt.xlabel("q[{}] - {} ticks".format(q_ind[0], q_steps[0]))
    plt.ylabel("q[{}] - {} ticks".format(q_ind[1], q_steps[1]))
    plt.title("Binary collision map")


def plot2DGridJacobian(J, q_ind, q_ranges, q_steps, title=""):
    grid_Jx = J[:,0].reshape(q_steps[1], q_steps[0])
    grid_Jy = J[:,1].reshape(q_steps[1], q_steps[0])

    fft_Jx = np.fft.fft2(grid_Jx)
    fft_Jy = np.fft.fft2(grid_Jy)

    plt.figure()
    plt.suptitle(title)
    plt.subplot(2,2,1)
    plt.imshow(grid_Jx, extent=q_ranges[0]+q_ranges[1], cmap=plt.cm.PiYG)
    plt.title("J_q{}".format(q_ind[0]))

    plt.subplot(2,2,3)
    plt.imshow(grid_Jy, extent=q_ranges[0]+q_ranges[1], cmap=plt.cm.PiYG)
    plt.title("J_q{}".format(q_ind[1]))

    plt.subplot(2,2,2)
    plt.imshow(np.log(np.abs(np.fft.fftshift(fft_Jx))), extent=q_ranges[0]+q_ranges[1])
    plt.title("J_q{} - FFT".format(q_ind[0]))

    plt.subplot(2,2,4)
    plt.imshow(np.log(np.abs(np.fft.fftshift(fft_Jy))), extent=q_ranges[0]+q_ranges[1])
    plt.title("J_q{} - FFT".format(q_ind[1]))


def plot3DDistance(col_map, q_ind, q_ranges, q_steps):
    col_map_pos = col_map[np.where(col_map[:,3]>0)]
    col_map_neg = col_map[np.where(col_map[:,3]<=0)]

    fig1 = plt.figure()
    # Positive and negative distances split apart
    ax10 = fig1.add_subplot(121, projection='3d')
    ax10.scatter(col_map_pos[:,0], col_map_pos[:,1], col_map_pos[:,2], c=col_map_pos[:,3], cmap=plt.cm.RdYlGn, vmin=np.min(col_map[:,3]))
    ax10.set_facecolor('xkcd:grey')
    ax10.set_xlabel("q[{}]".format(q_ind[0]))
    ax10.set_ylabel("q[{}]".format(q_ind[1]))
    ax10.set_zlabel("q[{}]".format(q_ind[2]))
    ax10.set_title("Distance > 0")

    ax11 = fig1.add_subplot(122, projection='3d')
    ax11.scatter(col_map_neg[:,0], col_map_neg[:,1], col_map_neg[:,2], c=col_map_neg[:,3], cmap=plt.cm.RdYlGn, vmax=np.max(col_map[:,3]))
    ax11.set_facecolor('xkcd:grey')  
    ax11.set_xlabel("q[{}]".format(q_ind[0]))
    ax11.set_ylabel("q[{}]".format(q_ind[1]))
    ax11.set_zlabel("q[{}]".format(q_ind[2]))  
    ax11.set_title("Distance < 0")

def plot3DJacobian(col_map, J, q_ind, q_ranges, q_steps):
    J0_pos, J0_neg = np.where(J[:,0] > 0), np.where(J[:,0] <0)
    J1_pos, J1_neg = np.where(J[:,1] > 0), np.where(J[:,1] <0) 
    J2_pos, J2_neg = np.where(J[:,2] > 0), np.where(J[:,2] <0) 

    fig2 = plt.figure()
    # Positive and negative jacobians split apart
    # J0
    ax20 = fig2.add_subplot(231, projection='3d')
    ax20.scatter(col_map[J0_pos,0], col_map[J0_pos,1], col_map[J0_pos,2], c=J[J0_pos][:,0], cmap=plt.cm.PiYG, vmin=np.min(J[:,0]))
    ax20.set_facecolor('xkcd:grey')
    ax20.set_title("J_q{} > 0".format(q_ind[0]))

    ax21 = fig2.add_subplot(234, projection='3d')
    ax21.scatter(col_map[J0_neg,0], col_map[J0_neg,1], col_map[J0_neg,2], c=J[J0_neg][:,0], cmap=plt.cm.PiYG, vmax=np.max(J[:,0]))
    ax21.set_facecolor('xkcd:grey')
    ax21.set_title("J_q{} < 0".format(q_ind[0]))

    # J1
    ax22 = fig2.add_subplot(232, projection='3d')
    ax22.scatter(col_map[J1_pos,0], col_map[J1_pos,1], col_map[J1_pos,2], c=J[J1_pos][:,1], cmap=plt.cm.PiYG, vmin=np.min(J[:,1]))
    ax22.set_facecolor('xkcd:grey')
    ax22.set_title("J_q{} > 0".format(q_ind[1]))

    ax23 = fig2.add_subplot(235, projection='3d')
    ax23.scatter(col_map[J1_neg,0], col_map[J1_neg,1], col_map[J1_neg,2], c=J[J1_neg][:,1], cmap=plt.cm.PiYG, vmax=np.max(J[:,1]))
    ax23.set_facecolor('xkcd:grey')
    ax23.set_title("J_q{} < 0".format(q_ind[1]))

    # J2
    ax24 = fig2.add_subplot(233, projection='3d')
    ax24.scatter(col_map[J2_pos,0], col_map[J2_pos,1], col_map[J2_pos,2], c=J[J2_pos][:,2], cmap=plt.cm.PiYG, vmin=np.min(J[:,2]))
    ax24.set_facecolor('xkcd:grey')
    ax24.set_title("J_q{} > 0".format(q_ind[2]))

    ax25 = fig2.add_subplot(236, projection='3d')
    ax25.scatter(col_map[J2_neg,0], col_map[J2_neg,1], col_map[J2_neg,2], c=J[J2_neg][:,2], cmap=plt.cm.PiYG, vmax=np.max(J[:,2]))
    ax25.set_facecolor('xkcd:grey')
    ax25.set_title("J_q{} < 0".format(q_ind[2]))


##### MAIN #####
if __name__ == "__main__":
    # execute only if run as a script
    robot, rmodel, rdata, gmodel, gdata = initSolo()
    ref_config = np.zeros(robot.q0.shape)

    x_rot_range = [-np.pi, np.pi]
    y_rot_range = [-np.pi, np.pi]
    knee_rot_range = [-np.pi, np.pi]    

    #q_ind = [7,8,9]
    q_ind = [7,8]
    #q_ranges = [x_rot_range, y_rot_range, knee_rot_range]
    q_ranges = [x_rot_range, y_rot_range]
    #q_steps = [20,20,100]
    q_steps = [500,500]

    # Example of boundary estimation between 2 known configuration planes
    '''
    bound_ref_dir = 0
    bound_ref_val0 = [0,-np.pi]
    bound_ref_val1 = [0, np.pi]
    
    bound0 = dichotomyBoundaryPlaneSampling(q_ind, bound_ref_dir, bound_ref_val0, q_ranges[1:], 25, 1e-9, ref_config, [0,1], rmodel, rdata, gmodel, gdata, extend_periodic=False)
    bound1 = dichotomyBoundaryPlaneSampling(q_ind, bound_ref_dir, bound_ref_val1, q_ranges[1:], 25, 1e-9, ref_config, [0,1], rmodel, rdata, gmodel, gdata, extend_periodic=False)
    bound = np.concatenate((bound0, bound1))
    '''
    # Load an existing sampled boundary
    #bound3d = np.load("./npy_data/3d_bound_ref_200x200.npy", allow_pickle=True)
    bound2d = np.load("./npy_data/2d_bound_ref_5000samp.npy", allow_pickle=True)
    bound = bound2d

    # Sample FCL distance
    #col_map = sampleRandomCollisionMap(ref_config, q_ind, q_ranges, 50000000, [0,1], rmodel, rdata, gmodel, gdata, computeDist=False)
    #col_map = sampleAroundBoundDistanceMap(ref_config, q_ind, q_ranges, 15000, bound, [0,1], rmodel, rdata, gmodel, gdata)
    col3d_map = sampleGridCollisionMap(ref_config, q_ind, q_ranges, q_steps, [0,1], rmodel, rdata, gmodel, gdata, computeDist=True)
    
    #np.save("./npy_data/datasets/3d/ref_gridSampling_3dDist_{}x{}x{}samples.npy".format(q_steps[0],q_steps[1],q_steps[2]), col_map)

    # Convert spatial distance to articular distance
    # Compute jacobian as well
    dist_metric = 'euclidean'
    col_map = spatialToArticular(col3d_map, bound, batch_size=100, metric=dist_metric)

    #np.save("./npy_data/datasets/3d/ref_gridSampling_articularDist_{}x{}x{}samples.npy".format(q_steps[0],q_steps[1],q_steps[2]), col_map)
    #J = getSampledJac(col_map, bound, batch_size=100, metric=dist_metric)

    # Plot the data
    #plot2DGridDistance(col_map, q_ind, q_ranges, q_steps)
    grid_data = col_map[:,2].reshape(q_steps[1], q_steps[0])
    grid3d_data = col3d_map[:,2].reshape(q_steps[1], q_steps[0])
    
    plt.figure()
    plt.subplot(3,1,3)
    plt.imshow(grid_data, extent=q_ranges[0]+q_ranges[1], cmap=plt.cm.RdYlGn, vmin=np.min(col_map[:,2]))
    plt.xlabel("q[{}] - {} ticks".format(q_ind[0], q_steps[0]))
    plt.ylabel("q[{}] - {} ticks".format(q_ind[1], q_steps[1]))
    plt.title("3D distance")

    plt.subplot(3,1,1)
    plt.imshow(grid3d_data, extent=q_ranges[0]+q_ranges[1], cmap=plt.cm.RdYlGn, vmin=np.min(col3d_map[:,2]))
    plt.xlabel("q[{}] - {} ticks".format(q_ind[0], q_steps[0]))
    plt.ylabel("q[{}] - {} ticks".format(q_ind[1], q_steps[1]))
    plt.title("Areticular distance")

    plt.subplot(3,1,2)
    plt.imshow(grid_data > 0, extent=q_ranges[0]+q_ranges[1])
    plt.xlabel("q[{}] - {} ticks".format(q_ind[0], q_steps[0]))
    plt.ylabel("q[{}] - {} ticks".format(q_ind[1], q_steps[1]))
    plt.title("Binary collision map")

    #plot3Ddata(col_map, q_ind, q_ranges, q_steps) #, J=J)
    plt.show()
    
    

    