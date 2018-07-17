#!/usr/bin/python

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt('auto-liftleg.csv', delimiter=',', skip_header=0, skip_footer=0)
data = data.transpose()

for i in range(len(data[2])):
    if data[2][i] < 0:
        data[2][i] = 160000

fig = plt.figure()
ax = fig.gca(projection='3d')

ax.set_title("Lift Peterb Cycles")
ax.set_xlim([0,2000])
ax.set_ylim([0,100])
ax.set_zlim([0,20000])
ax.set_xlabel("Spring Constant")
ax.set_ylabel("Dampening Constant")
ax.set_zlabel("Simulation Cycles")
ax.set_zlabel("Simulation Cycles")

ax.plot(data[0], data[1], data[2])

plt.show()

