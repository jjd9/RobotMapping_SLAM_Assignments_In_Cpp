import matplotlib.pyplot as plt
import numpy as np

import IPython

data = np.loadtxt(r"..\C++\SLAM_estimates.csv",delimiter=",")
data[data==0] = np.nan
robot = data[:,:2]
plt.plot(robot[:,0],robot[:,1],'ko',label='robot')

for j, i in enumerate(range(3,data.shape[1],2)):
    plt.plot(data[:,i],data[:,i+1],'o', label = 'landmark: '+ str(j))

plt.grid()
plt.xlabel("X Coord")
plt.ylabel("Y Coord")
plt.legend()
plt.show()
