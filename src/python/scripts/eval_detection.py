from solo_collisions_avoidance_control.solo_initialization import *
from solo_collisions_avoidance_control.solo_coll_wrapper_c import *

robot, rmodel, rdata, gmodel, gdata = initSolo(solo=False)

# Get the links geometries
base_link_geom = gmodel.getGeometryId("base_link_0")
FL_upper_leg_geom = gmodel.getGeometryId("FL_UPPER_LEG_0")
FL_lower_leg_geom = gmodel.getGeometryId("FL_LOWER_LEG_0")
FR_upper_leg_geom = gmodel.getGeometryId("FR_UPPER_LEG_0")
FR_lower_leg_geom = gmodel.getGeometryId("FR_LOWER_LEG_0")
HL_upper_leg_geom = gmodel.getGeometryId("HL_UPPER_LEG_0")
HL_lower_leg_geom = gmodel.getGeometryId("HL_LOWER_LEG_0")
HR_upper_leg_geom = gmodel.getGeometryId("HR_UPPER_LEG_0")
HR_lower_leg_geom = gmodel.getGeometryId("HR_LOWER_LEG_0")

gmodel.addCollisionPair(pio.CollisionPair(base_link_geom, FL_upper_leg_geom))
gmodel.addCollisionPair(pio.CollisionPair(base_link_geom, FL_lower_leg_geom))
gmodel.addCollisionPair(pio.CollisionPair(base_link_geom, FR_upper_leg_geom))
gmodel.addCollisionPair(pio.CollisionPair(base_link_geom, FR_lower_leg_geom))

gmodel.addCollisionPair(pio.CollisionPair(base_link_geom, HL_upper_leg_geom))
gmodel.addCollisionPair(pio.CollisionPair(base_link_geom, HL_lower_leg_geom))
gmodel.addCollisionPair(pio.CollisionPair(base_link_geom, HR_upper_leg_geom))
gmodel.addCollisionPair(pio.CollisionPair(base_link_geom, HR_lower_leg_geom))

gdata = gmodel.createData()

legs_FCL_detections = np.zeros(len(gmodel.collisionPairs))
legs_model_detections = np.zeros(len(gmodel.collisionPairs) - 8)
legs_false_positives = np.zeros(len(gmodel.collisionPairs) - 8)
legs_false_negatives = np.zeros(len(gmodel.collisionPairs) - 8)

shd_FCL_detections = np.zeros(4)
shd_model_detections = np.zeros(4)
shd_false_positives = np.zeros(4)
shd_false_negatives = np.zeros(4)

# Set up C collision model
nn_so_file = "/home/tnoel/stage/solo-collisions/compiled_c_lib/libcoll_nn_shd_knee_large.so"
nnCCollFun = CDLL(nn_so_file)
so_file = "/home/tnoel/stage/solo-collisions/compiled_c_lib/libcoll_legs12_witnessP.so"
cCollFun = CDLL(so_file)

nb_motors = 12
nb_pairs = 20

wrong_configs = []

for k in range(10000):
    q = robot.q0
    q = 2*np.pi*np.random.rand(12)

    # Compute all the collisions
    pio.computeCollisions(rmodel,rdata,gmodel,gdata,q,False)
    # C computation
    c_results = getLegsCollisionsResults(q, cCollFun, nb_motors, nb_pairs, witnessPoints=True)
    c_dist_legs = getLegsDistances(c_results, nb_motors, nb_pairs, witnessPoints=True)
    # shoulders
    c_shd_dist, c_shd_jac = getAllShouldersCollisionsResults(q, nnCCollFun, 3, offset=0.11) #offset with 3 inputs: 0.18 (small), 0.11 (large)"

    # Print the status of collision for all collision pairs

    ### legs pairs
    for k in range(len(gmodel.collisionPairs)): 
        cr = gdata.collisionResults[k]
        cp = gmodel.collisionPairs[k]
        #print("collision pair:",gmodel.geometryObjects[cp.first].name,",",gmodel.geometryObjects[cp.second].name,"- collision:","Yes" if cr.isCollision() else "No")

        if(cr.isCollision()):
            legs_FCL_detections[k] += 1

        

    for k in range(20):
        cr = gdata.collisionResults[k]
        cp = gmodel.collisionPairs[k]

        if(c_dist_legs[k] <= 0):
            legs_model_detections[k] += 1

        if(cr.isCollision() and c_dist_legs[k] > 0):
            legs_false_negatives[k] += 1
            wrong_configs.append(q)

        if((not cr.isCollision()) and c_dist_legs[k] <= 0):
            legs_false_positives[k] += 1

    ### Shoulders
    
    for k in range(4):
        
        cr_u = gdata.collisionResults[20+2*k]
        cp_u = gmodel.collisionPairs[20+2*k]
        
        #cr_l = gdata.collisionResults[20+2*k+1]
        #cp_l = gmodel.collisionPairs[20+2*k+1]

        cr = (cr_l.isCollision() or cr_u.isCollision())
        #cr = cr_u.isCollision()

        #print(cr, c_shd_dist[k])

        if(c_shd_dist[k] <= 0):
            shd_model_detections[k] += 1
        
        if(cr and (c_shd_dist[k] > 0)):
            shd_false_negatives[k] += 1
            wrong_configs.append(q)

        if((not cr) and (c_shd_dist[k] <= 0)):
            shd_false_positives[k] += 1
    
        