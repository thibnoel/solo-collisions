import pinocchio as pio
from example_robot_data import loadSolo
import hppfcl
import numpy as np
import matplotlib.pyplot as plt
import time
from scipy.spatial import distance
from numpy.linalg import norm
from collision_approx_fourier import evalApproxGradFFT


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
def sampleDistanceFromShoulderConfig(config, x_rot, y_rot, collisionPair, rmodel, rdata, gmodel, gdata):
    test_config = config.copy() # to change; config can be any valid config

    test_config[7] = x_rot
    test_config[8] = y_rot

    pio.computeDistances(rmodel, rdata, gmodel, gdata, test_config)
    # Get distance and collision results from collision data
    collisions_dist = gdata.distanceResults
    dist = collisions_dist[collisionPair].min_distance  # pair 0 is between the FL upper leg and the body, see initSolo()
    # Get nearest points on the meshes - FOR REF.
    #p1 = collisions_dist[collisionPair].getNearestPoint1() 
    #p2 = collisions_dist[collisionPair].getNearestPoint2() 

    return dist


# Sample FCL spatial distance on grid-aligned points
# Arguments :
#   - xsteps, ysteps : nb. of grid divisions in each direction
def sampleGridCollisionMap(config, x_rot_range, y_rot_range, xsteps, ysteps, collisionPair, rmodel, rdata, gmodel, gdata):
    col_map = []
    for i in range(ysteps):
        for j in range(xsteps):
            qx = x_rot_range[0] + j*(x_rot_range[1] - x_rot_range[0])/xsteps
            qy = y_rot_range[0] + i*(y_rot_range[1] - y_rot_range[0])/ysteps
            d = sampleDistanceFromShoulderConfig(config, qx, qy, collisionPair, rmodel, rdata, gmodel, gdata)

            col_map.append([qx, qy, d])
        print("Generated {} / {} dist. samples".format((i+1)*xsteps, xsteps*ysteps), end='\r')
    print('\n')
    col_map = np.array(col_map)
    dist_map = col_map.reshape(xsteps, ysteps, 3)
    dist_map = dist_map[:,:,2]
    return dist_map


# Samples the FCL spatial distance for random configurations
def sampleRandomCollisionMap(config, x_rot_range, y_rot_range, npoints, collisionPair, rmodel, rdata, gmodel, gdata):
    col_map = np.zeros((npoints, 3))
    k = 0
    for k in range(npoints):
        x_rot = x_rot_range[0] + np.random.rand()*(x_rot_range[1] - x_rot_range[0])
        y_rot = y_rot_range[0] + np.random.rand()*(y_rot_range[1] - y_rot_range[0])
        dist3D = sampleDistanceFromShoulderConfig(config, x_rot, y_rot, collisionPair, rmodel, rdata, gmodel, gdata)

        col_map[k,0] = x_rot
        col_map[k,1] = y_rot
        col_map[k,2] = dist3D
        if( (k+1) % 100 == 0):
            print("Generated {} / {} dist. samples".format(k+1, npoints), end='\r')
    print('\n')
    return col_map


# Returns a boundary configuration lying outside the collision zone
# Arguments :
#   - q_free, q_coll : reference configurations in the free zone and collision zone
#       --> q_bound will be returned as q_bound = q_free + t*(q_coll - q_free), t in [0,1]
#   - threshold : max articular distance to the boundary (precision of the boundary approx.)
def dichotomyBoundaryLoc(q_free, q_coll, robot_config, threshold, collisionPairs, rmodel, rdata, gmodel, gdata):
    qf = q_free.copy()
    qc = q_coll.copy()
    while((qf-qc).T@(qf-qc) > threshold):
        new_q = 0.5*(qc+qf)
        dist = []
        for collPair in collisionPairs :
            dist.append(sampleDistanceFromShoulderConfig(robot_config, new_q[0], new_q[1], collPair, rmodel, rdata, gmodel, gdata))
        d = np.min(dist)
        if(d>0):
            qf = new_q
        else:
            qc = new_q
    return qf # ensure the boundary points are always sampled in the free zone 


# Returns a sampling of the collision boundary
# Uses the previous methods with :
#   - free configurations : q = [0,:] 
#   - collision configurations : q = [-pi,:] and q = [pi, :]
def dichotomyBoundaryEstim(config, y_rot_range, nb_y_samples, threshold, collisionPair, rmodel, rdata, gmodel, gdata):
    y_arange = np.arange(y_rot_range[0], y_rot_range[1], step=(y_rot_range[1] - y_rot_range[0])/nb_y_samples)
    known_free = np.array([np.zeros(y_arange.shape[0]), y_arange]).T
    known_coll = np.array([-np.pi*np.ones(y_arange.shape[0]), y_arange]).T
    known_coll2 = np.array([np.pi*np.ones(y_arange.shape[0]), y_arange]).T

    bound = []
    for i in range(nb_y_samples):
        qf = known_free[i]
        qc = known_coll[i]
        qs = dichotomyBoundaryLoc(qf, qc, config, threshold, collisionPair, rmodel, rdata, gmodel, gdata)
        bound.append(qs)

        qf = known_free[i]
        qc = known_coll[i-nb_y_samples//10]
        qs = dichotomyBoundaryLoc(qf, qc, config, threshold, collisionPair, rmodel, rdata, gmodel, gdata)
        bound.append(qs)

        qf = known_free[i]
        qc = known_coll[(i+nb_y_samples//10)%nb_y_samples]
        qs = dichotomyBoundaryLoc(qf, qc, config, threshold, collisionPair, rmodel, rdata, gmodel, gdata)
        bound.append(qs)

        qf = known_free[i]
        qc = known_coll2[i]
        qs = dichotomyBoundaryLoc(qf, qc, config, threshold, collisionPair, rmodel, rdata, gmodel, gdata)
        bound.append(qs)

        qf = known_free[i]
        qc = known_coll2[i-nb_y_samples//10]
        qs = dichotomyBoundaryLoc(qf, qc, config, threshold, collisionPair, rmodel, rdata, gmodel, gdata)
        bound.append(qs)

        qf = known_free[i]
        qc = known_coll2[(i+nb_y_samples//10)%nb_y_samples]
        qs = dichotomyBoundaryLoc(qf, qc, config, threshold, collisionPair, rmodel, rdata, gmodel, gdata)
        bound.append(qs)

        if((i+1) % 10 == 0):
            print("Computed {} / {} boundary config.".format(6*(i+1), 6*nb_y_samples), end='\r')

    print('\n')

    bound = np.array(bound)
    # Accounting for periodicity and duplicating the sampled points around 
    x_offset1 = np.array([2*np.pi,0])
    x_offset2 = np.array([-2*np.pi,0])
    y_offset1 = np.array([0,2*np.pi])
    y_offset2 = np.array([0,-2*np.pi])
    bound = np.concatenate((bound, bound+x_offset1, bound+x_offset2, bound+y_offset1, bound+y_offset2))

    return bound


# Converts a spatial distance (as returned by FCL) to articular distance based on boundary configurations
# Samples : size (n,3), holding [qx, qy, fcl_dist]
def spatialToArticular(dist_samples, bounds):
    samples = dist_samples.copy()
    updated_dist = np.min(distance.cdist(bounds, samples[:,:2]), axis=0)
    updated_dist *= -1*(samples[:,2]<=0) + (samples[:,2] > 0)
    samples[:,2] = updated_dist
    return samples


# Computes the Jacobian of the articular distance as :
#   - q : current config
#   - w : witness collision config
#   - u = q-w
#   --> J = (1/u.T*u)*u = (1/d)*u
def getSampledJac(dist_samples, bounds):
    witness_config_ind = np.argmin(distance.cdist(bounds, dist_samples[:,:2]), axis=0)
    witness_config = bounds[witness_config_ind]
    dist_samples = spatialToArticular(dist_samples, bounds)
    J = (dist_samples[:,:2] - witness_config)
    J[:,0] /= dist_samples[:,2]
    J[:,1] /= dist_samples[:,2]
    return J

# Sample articular distance on randomly placed points
# Arguments :
#   - bound : list of boundary configurations to compute the distance against
#   - d_nb_points : nb. of samples
def sampleRandomArticularDistanceMap(config, x_rot_range, y_rot_range, d_nb_points, bound, collisionPair, rmodel, rdata, gmodel, gdata):
    rand_map = sampleRandomCollisionMap(config, x_rot_range, y_rot_range, d_nb_points, collisionPair, rmodel, rdata, gmodel, gdata)
    rand_map = spatialToArticular(rand_map, bound)
    J = getSampledJac(rand_map, bound)
    return rand_map, J


# Sample articular distance on grid-aligned points
# Arguments :
#   - bound : list of boundary configurations to compute the distance against
#   - xsteps, ysteps : nb. of grid divisions in each direction
def sampleGridArticularDistanceMap(config, x_rot_range, y_rot_range, xsteps, ysteps, bound, collisionPair, rmodel, rdata, gmodel, gdata):
    col_map = []
    for i in range(ysteps):
        for j in range(xsteps):
            qx = x_rot_range[0] + j*(x_rot_range[1] - x_rot_range[0])/xsteps
            qy = y_rot_range[0] + i*(y_rot_range[1] - y_rot_range[0])/ysteps
            d = sampleDistanceFromShoulderConfig(config, qx, qy, collisionPair, rmodel, rdata, gmodel, gdata)

            col_map.append([qx, qy, d])
        print("Generated {} / {} dist. samples".format((i+1)*xsteps, xsteps*ysteps), end='\r')
    print('\n')
    col_map = np.array(col_map)
    dist_map = spatialToArticular(col_map, bound)
    J = getSampledJac(col_map, bound)
    return dist_map, J


# Sample articular distance on randomly placed points, but with higher probability around the boudary configurations
# Arguments :
#   - bound : list of boundary configurations to compute the distance against
#   - d_nb_points : nb. of samples
def sampleAroundBoundDistanceMap(config, x_rot_range, y_rot_range, d_nb_points, bound, collisionPair, rmodel, rdata, gmodel, gdata):

    def sampleAround(point, min_dist, max_dist, l, nb_samples):
        points = []
        for k in range(nb_samples):
            val = np.random.rand()
            qdist = min_dist + max_dist*np.exp(-l*val)
            rand_dir = 2*np.pi*np.random.rand()
            qx, qy = point[0] + qdist*np.cos(rand_dir), point[1] + qdist*np.sin(rand_dir)
            d = sampleDistanceFromShoulderConfig(config, qx, qy, collisionPair, rmodel, rdata, gmodel, gdata)
            points.append([qx, qy, d])
        return np.array(points)    
    
    count = 0
    points = []
    while(count < d_nb_points):
        rand_ind = np.random.randint(0, int(bound.shape[0]/5))
        p = bound[rand_ind]
        new_samples = sampleAround(p, 1e-3, 0.5*np.pi, 5, 100)
        points.append(new_samples)
        count += 100
        print("Generated {} / {} dist. samples".format(count, d_nb_points), end='\r')
    print('\n')
    points = np.concatenate(points)
    points = spatialToArticular(points, bound)
    J = getSampledJac(points, bound)
    return points, J



if __name__ == "__main__":
    # execute only if run as a script
    robot, rmodel, rdata, gmodel, gdata = initSolo()
    x_rot_range = [-np.pi, np.pi]
    y_rot_range = [-np.pi, np.pi]
    knee_rot_range = [-np.pi, np.pi]    

    #bound = dichotomyBoundaryEstim(robot.q0, y_rot_range, 250, 1e-9, [0,1], rmodel, rdata, gmodel, gdata)

    #rand_map, J = sampleGridArticularDistanceMap(robot.q0, x_rot_range, y_rot_range, 100, 100, bound,  0, rmodel, rdata, gmodel, gdata)
    #grid_map = grid_map.reshape(100,100,3)
    #J = J.reshape(100,100,2)

    #rand_map, J = sampleAroundBoundDistanceMap(robot.q0, x_rot_range, y_rot_range, 10000, bound,0,rmodel, rdata, gmodel, gdata)
    #rand_map, J = sampleRandomArticularDistanceMap(robot.q0, x_rot_range, y_rot_range, 10000, bound, 0, rmodel, rdata, gmodel, gdata)

    #dist_fft = np.fft.fft2(grid_map[:,:,2])
    #Jx, Jy = evalApproxGradFFT(dist_fft)
    #Jx, Jy = np.fft.ifft2(Jx).real, np.fft.ifft2(Jy).real

    ref_config = np.zeros(robot.q0.shape)
    
    #gridmap0 = sampleGridCollisionMap(ref_config, x_rot_range, y_rot_range, 50, 50, 0, rmodel, rdata, gmodel, gdata)
    knee_steps = 1
    plt.figure()
    for k in range(knee_steps):
        ref_config[9] = knee_rot_range[0] + (knee_rot_range[1] - knee_rot_range[0])*k/knee_steps
        
        #gridmap1 = sampleGridCollisionMap(ref_config, x_rot_range, y_rot_range, 100, 100, 1, rmodel, rdata, gmodel, gdata)
        #mixed = np.minimum(gridmap0, gridmap1)

        bound = dichotomyBoundaryEstim(ref_config, y_rot_range, 250, 1e-9, [0,1], rmodel, rdata, gmodel, gdata)
        grid0, J = sampleGridArticularDistanceMap(ref_config, x_rot_range, y_rot_range, 100, 100, bound,  0, rmodel, rdata, gmodel, gdata)
        grid1, J = sampleGridArticularDistanceMap(ref_config, x_rot_range, y_rot_range, 100, 100, bound,  1, rmodel, rdata, gmodel, gdata)

        mixed = np.minimum(grid0[:,2], grid1[:,2])

        mixed = mixed.reshape(100,100)

        plt.subplot(1,knee_steps,k+1)
        plt.imshow(mixed, extent=x_rot_range+y_rot_range, origin="lower", cmap=plt.cm.RdYlGn)
        plt.title("Min(d0,d1) - knee_rot = {:.3f}".format(ref_config[9]))
    
    # Plot grid-like data
    '''
    plt.figure()
    plt.imshow(gridmap0, extent=x_rot_range+y_rot_range, origin="lower", cmap=plt.cm.RdYlGn)
    #plt.scatter(bound[:,0], bound[:,1], c='yellow')

    plt.figure()
    plt.imshow(gridmap1, extent=x_rot_range+y_rot_range, origin="lower", cmap=plt.cm.RdYlGn)
    '''

    #plt.figure()
    # Plot grid-like data
    '''
    plt.subplot(1,2,1)
    plt.imshow(J[:,:,0], extent=x_rot_range+y_rot_range, origin="lower", cmap=plt.cm.PiYG)
    plt.title("Jx - analytical")

    plt.subplot(1,2,2)
    plt.imshow(J[:,:,1], extent=x_rot_range+y_rot_range, origin="lower", cmap=plt.cm.PiYG)
    plt.title("Jy - analytical")

    plt.figure()
    plt.subplot(1,2,1)
    plt.imshow(Jx, extent=x_rot_range+y_rot_range, origin="lower", cmap=plt.cm.PiYG)
    plt.title("Jx - by FFT")

    plt.subplot(1,2,2)
    plt.imshow(Jy, extent=x_rot_range+y_rot_range, origin="lower", cmap=plt.cm.PiYG)
    plt.title("Jy - by FFT")
    '''

    # Plot non-gridded data
    '''
    plt.subplot(1,3,1)
    plt.scatter(colMap[:,0], colMap[:,1], c=colMap[:,2]>0, cmap=plt.cm.RdYlGn)
    plt.title("Articular Distance")

    plt.subplot(1,3,2)
    plt.scatter(colMap[:,0], colMap[:,1], c=J[:,0], cmap=plt.cm.PiYG)
    plt.title("Jx")

    plt.subplot(1,3,3)
    plt.scatter(colMap[:,0], colMap[:,1], c=J[:,1], cmap=plt.cm.PiYG)
    plt.title("Jy")
    '''
    plt.show()
