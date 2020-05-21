import pinocchio as pio
from example_robot_data import loadSolo
import hppfcl
import numpy as np
import matplotlib.pyplot as plt
import time
from numpy.linalg import norm

def initSolo():
    robot = loadSolo(solo=False)
    # Get robot model, data, and collision model
    rmodel = robot.model
    rdata  = rmodel.createData()
    gmodel = robot.collision_model

    # Get the base_link and FL_UPPER_LEG meshes
    #base_link_geom = gmodel.getGeometryId("base_link_0")
    #leg_geom = gmodel.getGeometryId("FL_UPPER_LEG_0")
    # Add the collision pair to the geometric model
    #gmodel.addCollisionPair(pio.CollisionPair(base_link_geom, leg_geom))

    gdata = gmodel.createData()
    return robot, rmodel, rdata, gmodel, gdata

def addCapsule(name, length, radius, linkFrameId, jointId, fMcaps, rmodel, gmodel, gui=None):
    caps = gmodel.addGeometryObject(pio.GeometryObject(name,
                                                     linkFrameId,jointId,
                                                     hppfcl.Capsule(radius,length),
                                                     fMcaps),rmodel)
    if(gui!=None):
        gui.addCapsule( "world/pinocchio/collisions/" + name, radius, length, [1,0,0,.1])
    return caps