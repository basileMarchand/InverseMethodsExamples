import numpy as np
import matplotlib.pyplot as plt

true_state = np.loadtxt("airfoil.dat")
ekf_state = np.loadtxt("state_ekf.dat")
ukf_state = np.loadtxt("state_ekf.dat")



plt.plot( true_state[:,0], true_state[:,1], label="dof 1 (ref)")
plt.plot( true_state[:,0], true_state[:,2], label="dof 2 (ref)")
plt.plot( ekf_state[:,0], ekf_state[:,1], label="dof 1 (ekf)")
plt.plot( ekf_state[:,0], ekf_state[:,2], label="dof 2 (ekf)")
plt.plot( ukf_state[:,0], ukf_state[:,1], label="dof 1 (ukf)")
plt.plot( ukf_state[:,0], ukf_state[:,2], label="dof 2 (ukf)")
plt.legend(loc='best')
plt.show()

