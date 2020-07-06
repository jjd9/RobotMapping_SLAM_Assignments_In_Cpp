import matplotlib.pyplot as plt
import numpy as np

odom = np.loadtxt("output/odom.csv", delimiter=",")
scan = np.loadtxt("output/scanmatch.csv", delimiter=",")
calib = np.loadtxt("output/calibrated.csv", delimiter=",")

plt.plot(odom[:,0],odom[:,1],label='Uncalibrated Odometry')
plt.plot(scan[:,0],scan[:,1],'g',label='Scan-Matching')
plt.plot(calib[:,0],calib[:,1],'r',label='Calibrated Odometry')
plt.legend()
plt.grid()
plt.show()
