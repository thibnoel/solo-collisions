from solo_shoulder_extended_collision_sampling import *
from example_robot_data import loadTalos

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
                    "arm_left_4_link_0",
                    "arm_left_5_link_0"]
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
    gmodel.addCollisionPair(pio.CollisionPair(geom_IDs[0], geom_IDs[4]))

    npairs = len(gmodel.collisionPairs)

    gdata = gmodel.createData()
    return robot, rmodel, rdata, gmodel, gdata, geom_names, npairs

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


q0_range = [0 - 2.4, 2*np.pi - 2.4]
q1_range = [0 - 2.4, 2*np.pi - 2.4]
q2_range = [0, 2*np.pi]

q_ind = [21,22,23]
q_ranges = [q0_range, q1_range, q2_range]
q_steps = [25,25, 9]

col_map = sampleGridCollisionMap(robot_config, q_ind, q_ranges, q_steps, [k for k in range(npairs)], rmodel, rdata, gmodel, gdata, computeDist=True)

plot3DDistance(col_map, q_ind, q_ranges, q_steps)
#plot2DGridDistance(col_map, q_ind, q_ranges, q_steps)
plt.show()