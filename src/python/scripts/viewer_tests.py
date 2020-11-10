from solo_collisions_avoidance_control.solo_initialization import *
from solo_collisions_avoidance_control.solo_coll_wrapper_c import *
from solo_collisions_avoidance_control.collisions_controller import *

robot, rmodel, rdata, gmodel, gdata = initSolo(solo=False)
robot.initViewer(True, viewer=gui)