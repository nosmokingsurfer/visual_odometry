from numpy import genfromtxt
import matplotlib.pyplot as plt

xyz = genfromtxt('../build/out.txt', delimiter=' ')

plt.plot(xyz[:,0], xyz[:,1])
plt.show()