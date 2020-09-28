import pinocchio as pio
from example_robot_data.robots_loader import *
from example_robot_data import *
import hppfcl
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from quadprog import solve_qp

#from solo12_collision_gradient import computeDistJacobian, computeDistJacobianFiniteDiff
#from collision_approx.collision_approx_pytorch import *


##### INITIALIZATION METHODS
# Copied from example robot data, remove free flyer base 
def loadSimplifiedSolo(solo=True):
    if solo:
        URDF_FILENAME = "solo.urdf"
    else:
        URDF_FILENAME = "solo12.urdf"
    SRDF_FILENAME = "solo.srdf"
    SRDF_SUBPATH = "/solo_description/srdf/" + SRDF_FILENAME
    URDF_SUBPATH = "/solo_description/robots/" + URDF_FILENAME
    modelPath = getModelPath(URDF_SUBPATH)
    """
    # Load URDF file
    robot = RobotWrapper.BuildFromURDF(modelPath + URDF_SUBPATH) #, [getVisualPath(modelPath)])
                                       #pinocchio.JointModelFreeFlyer())
    # Load SRDF file
    #robot.q0 = readParamsFromSrdf(robot.model, modelPath + SRDF_SUBPATH, False, False, "standing")
    # Add the free-flyer joint limits
    #addFreeFlyerJointLimits(robot.model)
    """
    # SIMPLIFIED URDF (capsules collisions)
    
    urdf_path = "/home/tnoel/stage/solo-collisions/urdf/"
    mesh_dir = "/opt/openrobots/share/example-robot-data/robots/solo_description"

    if solo:
        urdf_file = "solo8_simplified.urdf"
    else:
        urdf_file = "solo12_simplified.urdf"
    

    robot = RobotWrapper.BuildFromURDF(urdf_path + urdf_file, [mesh_dir])#, pio.JointModelFreeFlyer())
    robot.q0 = readParamsFromSrdf(robot.model, modelPath + SRDF_SUBPATH, False, False, "standing")
    #robot_config = robot.q0
    
    return robot

# Initialize SOLO model
def initSolo(solo=True):
    robot = loadSimplifiedSolo(solo=solo)
    # Get robot model, data, and collision model
    rmodel = robot.model
    rdata  = rmodel.createData()
    gmodel = robot.collision_model

    # Get the base_link and FL_UPPER_LEG geometries
    base_link_geom = gmodel.getGeometryId("base_link_0")
    FL_upper_leg_geom = gmodel.getGeometryId("FL_UPPER_LEG_0")
    FL_lower_leg_geom = gmodel.getGeometryId("FL_LOWER_LEG_0")
    FR_upper_leg_geom = gmodel.getGeometryId("FR_UPPER_LEG_0")
    FR_lower_leg_geom = gmodel.getGeometryId("FR_LOWER_LEG_0")
    HL_upper_leg_geom = gmodel.getGeometryId("HL_UPPER_LEG_0")
    HL_lower_leg_geom = gmodel.getGeometryId("HL_LOWER_LEG_0")
    HR_upper_leg_geom = gmodel.getGeometryId("HR_UPPER_LEG_0")
    HR_lower_leg_geom = gmodel.getGeometryId("HR_LOWER_LEG_0")

    legs_collGeoms = [FL_upper_leg_geom, 
                 FL_lower_leg_geom,
                 FR_upper_leg_geom, 
                 FR_lower_leg_geom,
                 HL_upper_leg_geom, 
                 HL_lower_leg_geom,
                 HR_upper_leg_geom, 
                 HR_lower_leg_geom]

    # Add the collision pairs to the geometric model
    gmodel.addCollisionPair(pio.CollisionPair(FL_upper_leg_geom, FR_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FL_upper_leg_geom, FR_lower_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FL_lower_leg_geom, FR_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FL_lower_leg_geom, FR_lower_leg_geom))

    #gmodel.addCollisionPair(pio.CollisionPair(FL_upper_leg_geom, HL_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FL_upper_leg_geom, HL_lower_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FL_lower_leg_geom, HL_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FL_lower_leg_geom, HL_lower_leg_geom))
   
    #gmodel.addCollisionPair(pio.CollisionPair(FL_upper_leg_geom, HR_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FL_upper_leg_geom, HR_lower_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FL_lower_leg_geom, HR_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FL_lower_leg_geom, HR_lower_leg_geom))

    #gmodel.addCollisionPair(pio.CollisionPair(FR_upper_leg_geom, HL_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FR_upper_leg_geom, HL_lower_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FR_lower_leg_geom, HL_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FR_lower_leg_geom, HL_lower_leg_geom))

    #gmodel.addCollisionPair(pio.CollisionPair(FR_upper_leg_geom, HR_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FR_upper_leg_geom, HR_lower_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FR_lower_leg_geom, HR_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(FR_lower_leg_geom, HR_lower_leg_geom))

    gmodel.addCollisionPair(pio.CollisionPair(HL_upper_leg_geom, HR_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(HL_upper_leg_geom, HR_lower_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(HL_lower_leg_geom, HR_upper_leg_geom))
    gmodel.addCollisionPair(pio.CollisionPair(HL_lower_leg_geom, HR_lower_leg_geom))

    gdata = gmodel.createData()
    return robot, rmodel, rdata, gmodel, gdata
