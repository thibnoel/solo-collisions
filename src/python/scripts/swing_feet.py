
import pinocchio as pin
import numpy as np
import time
from IPython import embed
import time
from timeit import default_timer as timer
from example_robot_data import load
robot = load('solo12')

#Inputs
dt = 0.001
feet_position_ref =     [0*np.array([0.1946,   0.16891, 0.0191028]),0.0*np.array([0.1946,  -0.16891, 0.0191028]),np.array([-0.1946,   0.16891, 0.0191028]),np.array([-0.1946,  -0.16891, 0.1191028])]
feet_velocity_ref =     [np.array([0,0,0]), np.array([0,0,0]), np.array([0,0,0]), np.array([0,0,0])]
feet_acceleration_ref = [np.array([0,0,0]), np.array([0,0,0]), np.array([0,0,0]), np.array([0,0,0])]
flag_in_contact = np.array([1,1,1,1])
base_orientation_ref = pin.utils.rpyToMatrix(0,0,0)
base_angularvelocity_ref = np.array([0,0,0])
base_angularacceleration_ref = np.array([0,0,0])
base_position_ref = np.array([0,0.0,0.235])
base_linearvelocity_ref = np.array([0,0,0])
base_linearacceleration_ref = np.array([0,0,0])

### Ref. parameters
amplitude_r = np.array([.08,-.09,.0])
freq_r = np.array([.6*np.pi,.6*np.pi,0*np.pi])
phase_r = np.array([ 0,np.pi/2,0] )

amplitude_l = np.array([.08,.09,.0])
freq_l = np.array([.4*np.pi,.5*np.pi,0*np.pi])
phase_l = np.array([ 0,np.pi/2,0] )

### Test. parameters
amplitude_r = np.array([.08,-.09,.0])
freq_r = np.array([.6*np.pi,.6*np.pi,0*np.pi])
phase_r = np.array([ 0,np.pi/2,0] )

amplitude_l = np.array([.08,.1,.0])
freq_l = np.array([.6*np.pi,.5*np.pi,0*np.pi])
phase_l = np.array([ 0,np.pi/2,0] )

### Shoulder coll test. parameters
amplitude_r = np.array([.08,-.08,.0])
freq_r = np.array([.6*np.pi,.6*np.pi,0*np.pi])
phase_r = np.array([ 0,np.pi/2,0] )

amplitude_l = np.array([.08,.1,.0])
freq_l = np.array([.6*np.pi,.5*np.pi,0*np.pi])
phase_l = np.array([ 0,np.pi/2,0] )

### Simult. coll test. parameters
amplitude_r = np.array([.09,-.1,.0])
freq_r = np.array([.6*np.pi,.6*np.pi,0*np.pi])
phase_r = np.array([ 0,np.pi/2,0] )

amplitude_l = np.array([.1,.08,.0])
freq_l = np.array([.4*np.pi,.4*np.pi,0*np.pi])
phase_l = np.array([ 0,np.pi/2,0] )


feet_position_ref = [
    lambda t: np.array([0.1946 - 0.03,   0.16891 + 0., 0.0191028]) + (1-np.cos(freq_r*t+phase_r))*amplitude_r,
    #np.array([-0.1946,   0.16891, 0.1191028]),
    lambda t: np.array([0.1946 + 0.,   -0.16891 - 0., 0.0191028]) + (1-np.cos(freq_l*t+phase_l))*amplitude_l,
    #lambda t: np.array([-0.1946,  -0.16891, 0.0191028])
    lambda t: np.array([-0.1946,   0.16891, 0.0191028]),
    lambda t: np.array([-0.1946,  -0.16891, 0.0191028]),
]
feet_velocity_ref = [
    lambda t: np.sin(freq_r*t)*freq_r*amplitude_r,
    lambda t: np.sin(freq_l*t)*freq_l*amplitude_l,
    #    lambda t: np.array([0,0,0.]),
    lambda t: np.array([0,0,0.]),
    lambda t: np.array([0,0,0.]),
]
feet_acceleration_ref = [
    lambda t: np.cos(freq_r*t)*freq_r**2*amplitude_r,
    lambda t: np.cos(freq_l*t)*freq_l**2*amplitude_l,
    #lambda t: np.array([0,0,0.]),
    lambda t: np.array([0,0,0.]),
    lambda t: np.array([0,0,0.]),
]
'''
feet_position_ref = [
    lambda t: np.array([0.1946,   0.16891, 0.0191028]),
    lambda t: np.array([0.1946 - 0.05,  -0.16891 + 0.05, 0.0191028 ]) + (1-np.cos(freq_r*t+phase_r))*amplitude_r,
    lambda t: np.array([-0.1946 + 0.0,   0.16891 + 0., 0.0191028]) ,
    #np.array([-0.1946,   0.16891, 0.1191028]),
    lambda t: np.array([-0.1946 + 0.05,   -0.16891 - 0., 0.0191028 ]) + (1-np.cos(freq_l*t+phase_l))*amplitude_l,
    #lambda t: np.array([-0.1946,  -0.16891, 0.0191028])
]
feet_velocity_ref = [
    lambda t: np.array([0,0,0.]),
    lambda t: np.sin(freq_r*t)*freq_r*amplitude_r,
    lambda t: np.array([0,0,0.]),
    lambda t: np.sin(freq_l*t)*freq_l*amplitude_l,
    #    lambda t: np.array([0,0,0.]),
]
feet_acceleration_ref = [
    lambda t: np.array([0,0,0.]),
    lambda t: np.cos(freq_r*t)*freq_r**2*amplitude_r,
    lambda t: np.array([0,0,0.]),
    lambda t: np.cos(freq_l*t)*freq_l**2*amplitude_l,
    #lambda t: np.array([0,0,0.]),
]
'''

USE_VIEWER = False

Kp_base_orientation = 1000
Kd_base_orientation = 2*np.sqrt(Kp_base_orientation)

Kp_base_position = 100
Kd_base_position = 2*np.sqrt(Kp_base_position)

Kp_flyingfeet = 10
Kd_flyingfeet = 2*np.sqrt(Kp_flyingfeet)

#pin.switchToNumpyMatrix()
if USE_VIEWER:
    robot.initViewer(loadModel=True)
    robot.viewer.gui.setRefreshIsSynchronous(False)
    robot.display(robot.q0)

q = robot.q0.copy()
dq = robot.v0.copy()
ddq = dq *0
#Get frame IDs
FL_FOOT_ID = robot.model.getFrameId('FL_FOOT')
FR_FOOT_ID = robot.model.getFrameId('FR_FOOT')
HL_FOOT_ID = robot.model.getFrameId('HL_FOOT')
HR_FOOT_ID = robot.model.getFrameId('HR_FOOT')
BASE_ID = robot.model.getFrameId('base_link')

def dinv(J,damping=1e-2):
    ''' Damped inverse '''
    U,S,V = np.linalg.svd(J)
    if damping==0:
        Sinv = 1/S
    else:
        Sinv = S/(S**2+damping**2)
    return (V.T*Sinv)@U.T


def computeSwingingFeetTorque():
    ### FEET
    Jfeet = []
    afeet = []
    pfeet = []
    pin.forwardKinematics(robot.model,robot.data,q,dq,np.zeros(robot.model.nv))
    pin.updateFramePlacements(robot.model,robot.data)
    
    for i_ee in range(4):
        idx = int(foot_ids[i_ee])

        pos = rdata.oMf[idx].translation
        nu = pin.getFrameVelocity(rmodel,rdata,idx,pin.LOCAL_WORLD_ALIGNED)
        ref = feet_position_ref[i_ee](i*dt)
        vref = feet_velocity_ref[i_ee](i*dt)
        aref = feet_acceleration_ref[i_ee](i*dt)
        
        J1 = pin.computeFrameJacobian(robot.model,robot.data,q,idx,pin.LOCAL_WORLD_ALIGNED)[:3]
        acc1 = -100*(pos-ref) - 20*(nu.linear-vref) + aref
        if flag_in_contact[i_ee] == 0:
            acc1 *= 0 # In contact = no feedback
        drift1 = np.zeros(3)
        drift1 += pin.getFrameAcceleration(rmodel,rdata,idx,pin.LOCAL_WORLD_ALIGNED).linear
        drift1 += np.cross(nu.angular,nu.linear)
        acc1 -= drift1

        Jfeet.append(J1)
        afeet.append(acc1)
        pfeet.append(pos)

    ### BASE
    idx = BASE_ID

    pos = rdata.oMf[idx].translation
    nu = pin.getFrameVelocity(rmodel,rdata,idx,pin.LOCAL_WORLD_ALIGNED)
    ref = base_position_ref
    Jbasis = pin.computeFrameJacobian(robot.model,robot.data,q,idx,pin.LOCAL_WORLD_ALIGNED)[:3]
    accbasis = -100*(pos-ref) - 20*nu.linear
    drift = np.zeros(3)
    drift += pin.getFrameAcceleration(rmodel,rdata,idx,pin.LOCAL_WORLD_ALIGNED).linear
    drift += np.cross(nu.angular,nu.linear)
    accbasis -= drift
    
    ### BASE ROTATION
    idx = BASE_ID

    rot = rdata.oMf[idx].rotation
    nu = pin.getFrameVelocity(rmodel,rdata,idx,pin.LOCAL_WORLD_ALIGNED)
    rotref = base_orientation_ref
    Jwbasis = pin.computeFrameJacobian(robot.model,robot.data,q,idx,pin.LOCAL_WORLD_ALIGNED)[3:]
    accwbasis = -100 * rotref @ pin.log3(rotref.T@rot) - 20*nu.angular
    drift = np.zeros(3)
    drift += pin.getFrameAcceleration(rmodel,rdata,idx,pin.LOCAL_WORLD_ALIGNED).angular
    accwbasis -= drift
    

    pinv = np.linalg.pinv
    inv = np.linalg.inv

    J = np.vstack(Jfeet)
    acc = np.concatenate(afeet)
    J=Jwbasis
    acc=accwbasis

    J = np.vstack(Jfeet+[Jbasis,Jwbasis])
    acc = np.concatenate(afeet+[accbasis,accwbasis])

    ddq = dinv(J) @ acc

    
    #print ("e1", e1)
    #print ("e2", e2)
    #print ("e3", e3)
    #pure integration
    #time.sleep(0.1)
    ddq=ddq

    return ddq, Jfeet, afeet, pfeet, pos, rot, ref, rotref


foot_ids = np.array([FL_FOOT_ID, FR_FOOT_ID, HL_FOOT_ID, HR_FOOT_ID])

hpos = []
hbas = []
rmodel=robot.model
rdata=robot.data

q_list = []
dq_list = []

ddq_list = []
tau_q_list = []
aq0 = np.zeros(robot.model.nv)

for i in range (120000):
    start = timer()
    """
    ### FEET
    Jfeet = []
    afeet = []
    pfeet = []
    pin.forwardKinematics(robot.model,robot.data,q,dq,np.zeros(robot.model.nv))
    pin.updateFramePlacements(robot.model,robot.data)
    
    for i_ee in range(4):
        idx = int(foot_ids[i_ee])

        pos = rdata.oMf[idx].translation
        nu = pin.getFrameVelocity(rmodel,rdata,idx,pin.LOCAL_WORLD_ALIGNED)
        ref = feet_position_ref[i_ee](i*dt)
        vref = feet_velocity_ref[i_ee](i*dt)
        aref = feet_acceleration_ref[i_ee](i*dt)
        
        J1 = pin.computeFrameJacobian(robot.model,robot.data,q,idx,pin.LOCAL_WORLD_ALIGNED)[:3]
        acc1 = -100*(pos-ref) - 20*(nu.linear-vref) + aref
        if flag_in_contact[i_ee] == 0:
            acc1 *= 0 # In contact = no feedback
        drift1 = np.zeros(3)
        drift1 += pin.getFrameAcceleration(rmodel,rdata,idx,pin.LOCAL_WORLD_ALIGNED).linear
        drift1 += np.cross(nu.angular,nu.linear)
        acc1 -= drift1

        Jfeet.append(J1)
        afeet.append(acc1)
        pfeet.append(pos)

    ### BASE
    idx = BASE_ID

    pos = rdata.oMf[idx].translation
    nu = pin.getFrameVelocity(rmodel,rdata,idx,pin.LOCAL_WORLD_ALIGNED)
    ref = base_position_ref
    Jbasis = pin.computeFrameJacobian(robot.model,robot.data,q,idx,pin.LOCAL_WORLD_ALIGNED)[:3]
    accbasis = -100*(pos-ref) - 20*nu.linear
    drift = np.zeros(3)
    drift += pin.getFrameAcceleration(rmodel,rdata,idx,pin.LOCAL_WORLD_ALIGNED).linear
    drift += np.cross(nu.angular,nu.linear)
    accbasis -= drift
    
    ### BASE ROTATION
    idx = BASE_ID

    rot = rdata.oMf[idx].rotation
    nu = pin.getFrameVelocity(rmodel,rdata,idx,pin.LOCAL_WORLD_ALIGNED)
    rotref = base_orientation_ref
    Jwbasis = pin.computeFrameJacobian(robot.model,robot.data,q,idx,pin.LOCAL_WORLD_ALIGNED)[3:]
    accwbasis = -100 * rotref @ pin.log3(rotref.T@rot) - 20*nu.angular
    drift = np.zeros(3)
    drift += pin.getFrameAcceleration(rmodel,rdata,idx,pin.LOCAL_WORLD_ALIGNED).angular
    accwbasis -= drift
    

    pinv = np.linalg.pinv
    inv = np.linalg.inv

    J = np.vstack(Jfeet)
    acc = np.concatenate(afeet)
    J=Jwbasis
    acc=accwbasis

    J = np.vstack(Jfeet+[Jbasis,Jwbasis])
    acc = np.concatenate(afeet+[accbasis,accwbasis])

    ddq = dinv(J) @ acc

    
    #print ("e1", e1)
    #print ("e2", e2)
    #print ("e3", e3)
    #pure integration
    #time.sleep(0.1)
    ddq=ddq
    end = timer()
    print(end - start)
    """
    ddq, Jfeet, afeet, pfeet, pos, rot, ref, rotref = computeSwingingFeetTorque()

    
    
    ddq_list.append(ddq)

    if(i > 0):
        tau_q = M@ddq + b
        tau_q_list.append(tau_q)

    #print (ddq)
    dq=dq+dt*ddq
    q=pin.integrate(robot.model,q,dq*dt)

    M = pin.crba(robot.model, robot.data, q)
    b = pin.rnea(robot.model, robot.data, q, dq, aq0)
    

    if USE_VIEWER:
        if (i%10==0):
            #print (i)
            robot.display(q)

    hpos.append(np.concatenate(pfeet))
    hbas.append(np.concatenate([pos-ref, pin.log3(rotref.T@rot)]))

    q_list.append(q)
    dq_list.append(dq)


import matplotlib.pylab as plt; plt.ion()
plt.subplot(411)
plt.plot([ p[:3] for p in hpos ])
plt.title('feet positions')
plt.subplot(412)
plt.plot([ p[3:6] for p in hpos ])
plt.subplot(413)
plt.plot([ p[6:9] for p in hpos ])
plt.subplot(414)
plt.plot([ p[9:] for p in hpos ])

plt.figure(2)
plt.subplot(211)
plt.title('Basis error')
plt.plot([ p[:3] for p in hbas ])
plt.subplot(212)
plt.plot([ p[3:6] for p in hbas ])

