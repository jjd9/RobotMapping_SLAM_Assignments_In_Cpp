import matplotlib.pyplot as plt
import numpy as np
from scipy.stats.distributions import chi2

def drawEllipse(C, x):
    alpha = 0.95
    # Calculate unscaled half axes
    sxx = C[0,0]
    syy = C[1,1]
    sxy = C[0,1]
    a = np.sqrt(0.5*(sxx + syy + np.sqrt((sxx-syy)**2 + 4*sxy**2)))   # always greater
    b = np.sqrt(0.5*sxx + syy - np.sqrt((sxx-syy)**2 + 4*sxy**2))   # always smaller

    # Remove imaginary parts in case of neg. definite C
    a = np.real(a)
    b = np.real(b)

    # Scaling in order to reflect specified probability
    a = a*np.sqrt(chi2.ppf(alpha,2))
    b = b*np.sqrt(chi2.ppf(alpha,2))

    # Look where the greater half axis belongs to
    if sxx < syy:
        swap = a
        a = b
        b = swap

    # Calculate inclination (numerically stable)
    if sxx != syy:
        angle = 0.5*np.arctan(2*sxy/(sxx-syy))
    elif sxy == 0:
        angle = 0     # angle doesn't matter
    elif sxy > 0:
        angle =  pi/4
    elif sxy < 0:
        angle = -pi/4

    # Draw ellipse
    # Constants
    NPOINTS = 100                   # point density or resolution

    # Compose point vector
    ivec = np.arange(0,2*np.pi,2*np.pi/NPOINTS)     # index vector
    p = np.zeros((2,ivec.size))

    p[0,:] = a*np.cos(ivec)           # 2 x n matrix which
    p[1,:] = b*np.sin(ivec)           # hold ellipse points

    # Translate and rotate
    xo = x[0]
    yo = x[1]
    R  = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    T  = np.array([xo, yo]).reshape(-1,1)*np.ones((1,ivec.size))
    p = R.dot(p) + T
    return p



mu0 = np.array([1,2])
C0 = np.eye(2)*0.1

mu1 = np.array([0.33, -0.67])
C1 = np.array([[0.336018, 0.150879],[0.150879,  2.83917]])

p0 = drawEllipse(C0, mu0)
p1 = drawEllipse(C1, mu1)

# Plot
plt.axis("equal")
plt.plot(p0[0,:],p0[1,:],'r',label='original dist')
plt.plot(mu0[[0]],mu0[[1]],'ro')
plt.plot(p1[0,:],p1[1,:],'b',label='new dist')
plt.plot(mu1[[0]],mu1[[1]],'bo')
plt.legend()
plt.grid()
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Unscented Transform")

plt.show()
