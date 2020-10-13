import numpy as np

# Compute a viscoelastic repulsive torque for a list of collisions results (distances + jacobians)
def computeRepulsiveTorque(q, vq, collDistances, collJacobians, dist_thresh=0.1, kp=0, kv=0, opposeJacIfNegDist=True):
    # Initialize repulsive torque
    tau_avoid = np.zeros(len(q))

    # Loop through the distance to check for threshold violation
    for i in range(len(collDistances)):
        J = collJacobians[i]
        d = collDistances[i]

        tau_rep = np.zeros(len(q))
        # If violation, compute viscoelastic repulsive torque along the collision jacobian
        if(d<0 and opposeJacIfNegDist):
            #d = 1e-3
            J = -J

        if(d < dist_thresh):
            tau_rep = -kp*(d - dist_thresh) - kv*J@vq        
        else : 
            tau_rep = 0    
        tau_avoid += tau_rep*J.T
        
    return tau_avoid


def computeEmergencyTorque(vq, kv):
    return -kv*vq


# Compute a condition to switch to the emergency behavior
# How to deal with v ? 
def emergencyCondition(collDistances, q, vq, tau_q, q_bounds, v_bounds, d_thresh, tau_thresh):
    if(np.min(q) < q_bounds[0] or np.max(q) > q_bounds[1] \
        or np.min(vq) < v_bounds[0] or np.max(vq) > v_bounds[1] \
        or np.min(collDistances) < d_thresh \
        or np.max(np.abs(tau_q)) > tau_thresh ):
        return True
    return False