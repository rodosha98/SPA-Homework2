import numpy as np
import matplotlib.pyplot as plt

# Data pre-processing
data = np.loadtxt('case4.txt', delimiter=',', unpack=True)
time, z = data
print (z)
plt.scatter(time, z, c='red', s=1)
plt.xlabel('time')
plt.ylabel('x coordinate')
plt.show()

# Initial parameters
n = np.shape(z)[0]
print(np.mean(z))
mu_psi, sigma_psi = 0, 10
mu_eta, sigma_eta = np.mean(z), np.sqrt(np.var(z))


# Kalman Filter
x_opt = np.zeros(n)
e_opt = np.zeros(n)
time_t = np.zeros(n)
K = np.zeros(n)

x_opt[0] = z[0]
e_opt[0] = sigma_eta

for t in range(0, n - 1):
    e_opt[t + 1] = np.sqrt((sigma_eta ** 2) * (e_opt[t]**2 + sigma_psi**2)/ (sigma_eta**2 + e_opt[t]**2 + sigma_psi**2))
    K[t+1] = (e_opt[t+1]**2) / sigma_eta**2
    x_opt[t+1] = x_opt[t]*(1 - K[t+1]) + K[t+1]*z[t+1]
    time_t[t] = t


plt.scatter(time_t, x_opt, c='blue', s = 1)
plt.scatter(time_t, z, c='yellow', s = 0.5)
plt.show()
plt.scatter(time_t, K, c='green', s = 1)
plt.show()


#Simplified Kalman Filter
x_optst = np.zeros(n)
K_stab = K[1000]
x_optst[0] = z[0]
for t in range(0, n - 1):
    x_optst[t + 1] = x_optst[t] * (1 - K_stab) + K_stab * z[t + 1]

plt.scatter(time_t, x_optst, c='red', s = 1)
plt.scatter(time_t, x_opt, c='blue', s = 1)
plt.scatter(time_t, z, c='yellow', s = 0.5)
plt.show()


