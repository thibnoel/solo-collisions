import numpy as np 
import matplotlib.pyplot as plt
from solo12_collisions_utils import followBoundary

# Load the collision map from file
col_map_file = './npy_data/collision_map_centered_res200.npy'
col_map = np.load(col_map_file, allow_pickle=True)

traj1 = followBoundary(col_map)
traj2 = followBoundary(col_map, first_dir=2)
traj2 = [traj2[-i] for i in range(len(traj2))]

traj1X = np.array([t[0] for t in traj1])
traj1Y = np.array([t[1] for t in traj1])

traj2X = np.array([t[0] for t in traj2])
#traj2Y = np.array([t[1] for t in traj2])
traj2X = np.concatenate([traj2X, traj2X + len(traj2X), traj2X + 2*len(traj2X)])
traj2Y = np.array(3*[t[1] for t in traj2])

def approxFourier(trajX, trajY, Nh, plot=True):
    complexTraj = np.array(trajX + 1j*trajY)
    period = len(complexTraj)
    time = np.array([i for i in range(period)])

    def cn(n):
        c = complexTraj*np.exp(-1j*2*n*np.pi*time/period)
        return c.sum()/c.size

    def f(x, Nh):
        f = np.array([cn(i)*np.exp(1j*2*i*np.pi*x/period) for i in range(-Nh-1,Nh+1)])
        return f.sum()

    traj_est = np.array([f(t,Nh) for t in time])
    #plt.figure()
    if(plot):
        plt.plot(complexTraj.real - complexTraj.real[int(len(complexTraj)/3)], complexTraj.imag, 
                                                                linewidth=2, 
                                                                c='limegreen')
        plt.plot(traj_est.real[10:-10] - complexTraj.real[int(len(complexTraj)/3)], traj_est.imag[10:-10], 'r-', linewidth=2,
                                                                label="Fourier series - {} harmonics".format(Nh))
        #plt.plot(trajX, trajY_est)
    return traj_est

def approxPolynom(trajX, trajY, deg, plot=True):
    polynCoeffs = np.polyfit(trajX, trajY, deg)
    polynEval = np.poly1d(polynCoeffs)

    if(plot):
        #plt.figure()
        #plt.plot(trajX, trajY)
        plt.plot(trajX, polynEval(trajX),linewidth=2, c='yellow',label="Polynom - deg. {}".format(deg))
    return [trajX, polynEval(trajX)]

#print(traj2)
#plt.subplot(2,2,1)
#approxFourier(traj1, 50, plot=False)
#plt.subplot(2,2,2)
#approxPolynom(traj1, 10, plot=False)
#plt.subplot(2,2,3)
plt.figure()
plt.imshow(col_map)
polynTraj1 = approxPolynom(traj1X, traj1Y, 50, plot=True)
traj1X = np.concatenate([traj1X, traj1X + len(traj1X), traj1X + 2*len(traj1X)])
traj1Y = np.array(3*[t[1] for t in traj1])
fourierTraj1 = approxFourier(traj1X, traj1Y, 100, plot=True)
#fourierTraj2 = approxFourier(traj2X, traj2Y, 100, plot=True)
#plt.subplot(2,2,4)

#polynTraj2 = approxPolynom(traj2, 10, plot=True)
plt.legend()
plt.title('Collision boundary approximation')

plt.figure()
plt.grid(True)
plt.plot(traj1X, traj1Y + 2*len(col_map)/4)
plt.plot(fourierTraj1.real[10:-10], fourierTraj1.imag[10:-10] + len(col_map)/4)
plt.plot(polynTraj1[0] + fourierTraj1.real[int(len(fourierTraj1)/3)], polynTraj1[1])
plt.show()