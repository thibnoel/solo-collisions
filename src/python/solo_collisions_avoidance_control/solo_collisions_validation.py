import numpy as np

from solo_collisions_avoidance_control.solo_initialization import *
from solo_collisions_avoidance_control.solo_coll_wrapper_fcl import *
from solo_collisions_avoidance_control.solo_coll_wrapper_c import *


def compareDistances(fcl_dist, c_dist, max_err=1e-3):
    err_ind = []
    for k in range(len(fcl_dist)):
        err = np.abs(fcl_dist[k] - c_dist[k])
        if(err > max_err):
            err_ind.append(k)
    
    if(len(err_ind)>0):
        print("Distances errors !\nFailing pairs : ")
        print(err_ind)
    else:
        print("Distances OK")
    
    return err_ind


def compareJacobian(fcl_jac, c_jac, max_err=1e-3):
    err_ind = []
    for k in range(len(fcl_jac)):
        err = np.abs(fcl_jac[k] - c_jac[k])
        if(err > max_err):
            err_ind.append(k)
    '''
    if(len(err_ind)>0):
        print("Jacobian errors !\nFailing ind. : ")
        print(err_ind)
    else:
        print("Jacobian OK")
    '''
    return err_ind

def compareJacobians(fcl_jlist, c_jlist, max_err=1e-3):
    err_pairs = []
    for k in range(len(fcl_jlist)):
        err_ind = compareJacobian(fcl_jlist[k], c_jlist[k], max_err=max_err)
        if(len(err_ind) > 0):
            err_pairs.append([k,err_ind])
    
    if(len(err_pairs)>0):
        print("Jacobian errors !\nFailing pairs and corresponding q indices : ")
        for i in range(len(err_pairs)):
            print(err_pairs[i][0],' : ', err_pairs[i][1]) 
    else:
        print("Jacobian OK")


robot, rmodel, rdata, gmodel, gdata = initSolo(solo=False)

nb_motors = 12
nb_pairs = 20
wPoints = True

q = np.random.random(nb_motors)


#so_file = "/home/tnoel/stage/solo-collisions/compiled_c_lib/libcoll_legs8_witnessP.so"
so_file = "/home/tnoel/stage/solo-collisions/compiled_c_lib/libcoll_legs12_witnessP.so"

cCollFun = CDLL(so_file)

# Get results for the legs from FCL
legs_dist, Jlegs, legs_pairs, witnessPoints = compute_legs_Jdist_avoidance(q, rmodel, rdata, gmodel, gdata)
legs_dist_FD, Jlegs_FD, legs_pairs, witnessPoints = compute_legs_Jdist_avoidance(q, rmodel, rdata, gmodel, gdata, FD=True)

### Get results from C generated code
# Independent call to feed the logs
#c_results = getLegsCollisionsResults8(q, cCollFun)
c_results = getLegsCollisionsResults(q, cCollFun, nb_motors, nb_pairs, witnessPoints=wPoints)
c_dist_legs = getLegsDistances(c_results, nb_motors, nb_pairs, witnessPoints=wPoints)
c_Jlegs = getLegsJacobians(c_results, nb_motors, nb_pairs, witnessPoints=wPoints)

c_wpoints = getLegsWitnessPoints(c_results, nb_motors, nb_pairs)

max_err = 1e-6

compareDistances(legs_dist, c_dist_legs)
compareJacobians(Jlegs, c_Jlegs)

#compareDistances(legs_dist, legs_dist_FD, max_err=max_err)
#compareJacobians(Jlegs, Jlegs_FD, max_err=max_err)