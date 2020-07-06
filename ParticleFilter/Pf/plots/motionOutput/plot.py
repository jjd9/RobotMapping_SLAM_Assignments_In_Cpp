import matplotlib.pyplot as plt
import numpy as np
import glob



files = glob.glob("csv/*.csv")
fileNumbers = [int(f[f.find("=")+1:f.find(".")]) for f in files]
files = [x for _, x in sorted(zip(fileNumbers, files))]

path = []
prevParticles = None
for f in files:
    particles = np.loadtxt(f,delimiter=",")
    path.append(np.mean(particles,axis=0))

    fig,ax=plt.subplots()
    if prevParticles is not None:
        ax.plot(prevParticles[:,0],prevParticles[:,1],'ro',markersize=3)
        ax.plot(np.array(path)[:,0],np.array(path)[:,1],'k-',markersize=3)

    ax.plot(particles[:,0],particles[:,1],'o',markersize=3)
    plt.savefig(f.replace("csv","png"))
    plt.close()
    prevParticles = particles.copy()
