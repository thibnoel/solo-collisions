from solo_collisions_avoidance_control.solo_initialization import *
from solo_collisions_avoidance_control.solo_coll_wrapper_c import *
from solo_collisions_avoidance_control.collisions_controller import *

robot, rmodel, rdata, gmodel, gdata = initSolo(solo=False)

USE_VIEWER = False
shoulder_ind = 0 # FL: 0, FR: 1, HL: 2, HR: 3

def sampleCircle(q_center, q_rad, nb_samples=3000):
    circle_configs = np.zeros((nb_samples, 2))
    for k in range(nb_samples):
        t = 2*np.pi*k*1./nb_samples
        q0 = q_center[0] + q_rad*np.cos(t)
        q1 = q_center[1] + q_rad*np.sin(t)

        circle_configs[k] = [q0, q1]
    return circle_configs

if USE_VIEWER:
    robot.initViewer(loadModel=True)
    robot.viewer.gui.setRefreshIsSynchronous(False)
    robot.display(robot.q0)

q_list_circ = sampleCircle([1.2,0], 1.5)

q_list_circ_offs = np.vstack((q_list_circ[1:], q_list_circ[0]))

vq_list_circ = q_list_circ_offs - q_list_circ

q = robot.q0

q_ref = np.zeros((120000,12))
dq_ref = np.zeros((120000,12))

for k in range(120000):
    q[3*shoulder_ind:3*shoulder_ind + 2] = q_list_circ[k%len(q_list_circ)]
    #time.sleep(0.001)
    #robot.display(q)

    q_ref[k,0:2] = q_list_circ[k%len(q_list_circ)]
    dq_ref[k,0:2] = vq_list_circ[k%len(vq_list_circ)]

