import matplotlib.pyplot as plt
import numpy as np

gridmap = np.loadtxt("GridMap.csv", delimiter=",")
path = np.loadtxt("RobotPath.csv", delimiter=",")

print("Grid size: ", gridmap.shape)
plt.imshow(gridmap, cmap=plt.cm.binary)
plt.plot(path[:,1],path[:,0],'r',lw=1)
plt.show()
