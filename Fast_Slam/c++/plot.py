import matplotlib.pyplot as plt
import numpy as np

data = np.loadtxt("Particles.csv", delimiter=",")
all_paths = np.loadtxt("Path.csv", delimiter=",")
world = np.loadtxt("data/world.dat")

labels = data[:, 0]
positions = data[:, 1:]

avgPath = None
for i in np.unique(all_paths[:, 0]):
    path = all_paths[all_paths[:, 0] == i]
    if avgPath is None:
        avgPath = path.copy()
    else:
        avgPath += path
    plt.plot(path[:, 1], path[:, 2], 'b', alpha=0.05)
avgPath /= 100.0
plt.plot(avgPath[:, 1], avgPath[:, 2], 'r', label='Robot path')


plt.scatter(positions[:, 0], positions[:, 1], c=labels, s=3, cmap='jet')
plt.plot(world[:, 1], world[:, 2], 'ko', label='Landmark position')

plt.grid()
plt.xlabel("X Coord")
plt.ylabel("Y Coord")
plt.legend()
plt.show()
