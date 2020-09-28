import pinocchio as pio
import hppfcl
import numpy as np
import time
from scipy.spatial import distance
from numpy.linalg import norm


# Evaluates the collision distance (mesh-to-mesh) with FCL for a given configuration
# Reference connfig is the configuration in which the robot should be tested for the non-sampled DoF
# If multiple pairs are given, returns the min. of them all
def sampleFCLDistanceFromConfig(ref_config, q_ind, q_val, collisionPairs, rmodel, rdata, gmodel, gdata, computeDist=True):
    test_config = ref_config.copy() 

    for k in range(len(q_ind)):
        test_config[q_ind[k]] = q_val[k]

    # Get distance and collision results from collision data
    if(computeDist):
        pio.computeDistances(rmodel, rdata, gmodel, gdata, test_config)
        collisions_dist = gdata.distanceResults
    else:
        pio.computeCollisions(rmodel, rdata, gmodel, gdata, test_config, True)
        collisions_dist = gdata.collisionResults
        
    distances = []
    for c in collisionPairs:
        # Pairs are taken in the order they were added to the geometric model of the robot
        if(computeDist):
            distances.append(collisions_dist[c].min_distance)
        else:
            distances.append(float(not collisions_dist[c].isCollision()))
    dist = np.min(distances)

    return np.min(distances)


# Wraps the previous method for a list of configurations
def sampleFCLDistanceFromConfigList(ref_config, q_ind, q_list, collisionPairs, rmodel, rdata, gmodel, gdata, computeDist=True, verbose=True):
    col_map = []

    for k in range(len(q_list)):
        q = q_list[k]
        d = sampleFCLDistanceFromConfig(ref_config, q_ind, q, collisionPairs, rmodel, rdata, gmodel, gdata, computeDist=computeDist)

        result = []
        result.extend(q)
        result.append(d)
        col_map.append(result)

        if(verbose):
            if(k%100 == 0):
                print("<FCL DIST. SAMPLING> Generated {} / {} dist. samples".format(k, len(q_list)), end='\r')
    if(verbose):
        print("<FCL DIST. SAMPLING> Generated {} / {} dist. samples".format(k+1, len(q_list)), end='\r')
        print('')

    col_map = np.array(col_map)
    return col_map


# Generate a given number of random n-D configurations
def generateRandomConfigs(nq, q_ind, q_ranges):
    q_list = []
    for k in range(nq):
        rand_q = []
        for i in range(len(q_ind)):
            rand_q.append(q_ranges[i][0] + np.random.rand()*(q_ranges[i][1] - q_ranges[i][0]))
        q_list.append(rand_q)

    return q_list


# Generate a given n-D grid of configurations in articular space
def generateGridConfigs(q_ind, q_ranges, q_steps):

    def qFromInd(index):
        q = []
        for k in range(len(index)):
            q.append(q_ranges[k][0] + index[k]*(q_ranges[k][1] - q_ranges[k][0])/q_steps[k])
        return q

    def updateGridInd(curr_index, k):
        if(k>=len(curr_index)):
            return curr_index
        if(curr_index[k] < q_steps[k] - 1):
            curr_index[k] += 1
            return curr_index
        else:
            curr_index[k] = 0
            curr_index = updateGridInd(curr_index, k+1)
            return curr_index
    
    q_list = []
    nb_samples = np.array([q_steps]).prod()
    
    counter = 0
    ind = [0]*len(q_ranges)
    
    while(counter < nb_samples):
        q = qFromInd(ind)
        # Update current grid index
        ind = updateGridInd(ind,0)
        counter += 1
        q_list.append(q)

    return q_list


# Returns a boundary configuration lying just outside the collision zone
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


def boundaryRandomSapling(q_ind, q_ranges, nb_bound, threshold, ref_config, collisionPairs, rmodel, rdata, gmodel, gdata, extend_periodic=False):
    bound = []
    while(len(bound) < nb_bound):
        rand_q = generateRandomConfigs(100, q_ind, q_ranges)
        dist = sampleFCLDistanceFromConfigList(ref_config, q_ind, rand_q, collisionPairs, rmodel, rdata, gmodel, gdata, computeDist=False, verbose=False)

        q_free_list = dist[np.where(dist[:,-1] > 0)][:,:-1]
        q_coll_list = dist[np.where(dist[:,-1] <= 0)][:,:-1]

        while(len(q_free_list) > 0 and len(q_coll_list) > 0 and len(bound) < nb_bound):
            f_ind = np.random.randint(0,len(q_free_list))
            c_ind = np.random.randint(0,len(q_coll_list))

            qs = dichotomyBoundaryLoc(q_free_list[f_ind], q_coll_list[c_ind], threshold,ref_config, q_ind, collisionPairs, rmodel, rdata, gmodel, gdata)
            bound.append(qs)

            q_free_list = np.delete(q_free_list, f_ind, axis=0)
            q_coll_list = np.delete(q_coll_list, c_ind, axis=0)
        
            if(len(bound) % 10 == 0):
                print("<BOUNDARY SAMPLING> Computed {} / {} boundary config.".format(len(bound), nb_bound), end='\r')
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


# Returns the boundary sampled between 2 reference planes aligned with and offset in a direction of q
# 1 should contain only collision configurations, the other one only free configurations
# !!! only works for special cases (depending on boundary shape)
def boundaryPlaneSampling(q_ind, q_ref_dir, q_ref_vals, q_ranges, q_res, threshold, ref_config, collisionPairs, rmodel, rdata, gmodel, gdata, extend_periodic=False):
    
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

    q_sample_ind = q_ind[1:]
    q_steps = [q_res]*len(q_sample_ind)


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
    dim = bounds.shape[1]
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
    return np.hstack((dist_samples[:,:-1],np.concatenate(full_J)))