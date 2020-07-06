import matplotlib.pyplot as plt
import numpy as np

bpos = np.loadtxt("bpos.csv", delimiter=",")
apos = np.loadtxt("apos.csv", delimiter=",")

plt.plot(bpos[:,0],bpos[:,1],'ro',markersize=3)
plt.plot(apos[:,0],apos[:,1],'o',markersize=3)

plt.show()
