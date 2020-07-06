import matplotlib.pyplot as plt
import pandas as pd

data = pd.read_csv("SLAM_estimates.csv").values

robot = data[:,:2]
plt.plot(robot[:,0],robot[:,1],'k-o',label='robot')

for j, i in enumerate(range(3,data.shape[1],2)):
    print(i)
    plt.plot(data[:,i],data[:,i+1],'o', label = 'landmark: '+ str(j))

plt.grid()
plt.xlabel("X Coord")
plt.ylabel("Y Coord")
plt.legend()
plt.show()
