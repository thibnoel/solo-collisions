import numpy as np 
import matplotlib.pyplot as plt
from solo12_collisions_utils import followBoundary

# Load the collision map from file
col_map_file = './collision_map_centered_res100.npy'
col_map = np.load(col_map_file, allow_pickle=True)
traj = followBoundary(col_map)


def approxFourier(traj, Nh):
    trajX = np.array([t[0] for t in traj])
    trajX = np.concatenate([trajX, trajX + len(trajX), trajX + 2*len(trajX)])
    trajY = np.array(3*[t[1] for t in traj])
    period = len(col_map)

    def cn(n):
        c = trajY*np.exp(-1j*2*n*np.pi*trajX/period)
        return c.sum()/c.size

    def f(x, Nh):
        f = np.array([2*cn(i)*np.exp(1j*2*i*np.pi*x/period) for i in range(1,Nh+1)])
        return f.sum()

    trajY_est = np.array([f(x,Nh).real for x in trajX])

    plt.figure()
    plt.title("Fourier series approx. with {} harmonics".format(Nh))
    plt.plot(trajX, trajY)
    plt.plot(trajX, trajY_est)

def approxPolynom(traj, deg):
    trajX = np.array([t[0] for t in traj])
    trajX = np.concatenate([trajX, trajX + len(trajX), trajX + 2*len(trajX)])
    trajY = np.array(3*[t[1] for t in traj])

    polynCoeffs = np.polyfit(trajX, trajY, deg)
    polynEval = np.poly1d(polynCoeffs)

    plt.figure()
    plt.title("Polynomial approx. of deg. {}".format(deg))
    plt.plot(trajX, trajY)
    plt.plot(trajX, polynEval(trajX))


approxFourier(traj,200)
approxPolynom(traj, 50)
plt.show()