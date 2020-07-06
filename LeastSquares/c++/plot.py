import matplotlib.pyplot as plt
import numpy as np

poses = np.loadtxt("PoseData.csv", delimiter=",")
landmarks = np.loadtxt("LandmarkData.csv", delimiter=",")

plt.plot(landmarks[:,0],landmarks[:,1],'ko',markersize=3)
plt.plot(poses[:,0],poses[:,1],'ro',markersize=1)

plt.grid()
plt.xlabel("X Coord")
plt.ylabel("Y Coord")
plt.legend()
plt.show()
