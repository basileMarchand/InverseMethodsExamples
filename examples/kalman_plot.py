import numpy as np
import matplotlib.pyplot as plt

true_state = np.loadtxt("airfoil.dat")
ekf_state = np.loadtxt("state_ekf.dat")
ukf_state = np.loadtxt("state_ukf.dat")



plt.plot( true_state[:,0], true_state[:,1], label="dof 1 (ref)")
plt.plot( true_state[:,0], true_state[:,2], label="dof 2 (ref)")
plt.plot( ekf_state[:,0], ekf_state[:,1], label="dof 1 (ekf)")
plt.plot( ekf_state[:,0], ekf_state[:,2], label="dof 2 (ekf)")
plt.plot( ukf_state[:,0], ukf_state[:,1], label="dof 1 (ukf)")
plt.plot( ukf_state[:,0], ukf_state[:,2], label="dof 2 (ukf)")
plt.legend(loc='best')
plt.title("Airfoil State")




t_ratio = int( (true_state[:,0].shape[0]-1) / (ekf_state[:,0].shape[0]-1) )

ukf_err1 = np.abs(( true_state[::t_ratio,1] - ukf_state[:,1] )/true_state[::t_ratio,1].max())
ukf_err2 = np.abs(( true_state[::t_ratio,1] - ukf_state[:,1] )/true_state[::t_ratio,2].max())
ekf_err1 = np.abs(( true_state[::t_ratio,1] - ekf_state[:,1] )/true_state[::t_ratio,1].max())
ekf_err2 = np.abs(( true_state[::t_ratio,1] - ekf_state[:,1] )/true_state[::t_ratio,2].max())

fix, ax = plt.subplots(2)
ax[0].semilogy( true_state[::t_ratio,0], ukf_err1)
ax[0].semilogy( true_state[::t_ratio,0], ukf_err2)

ax[1].semilogy( true_state[::t_ratio,0], ekf_err1)
ax[1].semilogy( true_state[::t_ratio,0], ekf_err2)
plt.show()
