#!/usr/bin/python

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt('foo.csv', delimiter=',', skip_header=0, skip_footer=0, dtype=float)
data = data.transpose()

fig = plt.figure()
ax = fig.gca(projection='3d')

ax.set_title("I love to walk")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

lasti = 0
i = 0
lastdata = None


while i < len(data[0]):
    while i < len(data[0]) and  data[0][i] == lastdata:
        i += 1
    if(i-1 >= 0):
	    ax.plot((data[2][lasti:i]), (data[3][lasti:i]) , (data[4][lasti:i]) ,'o' , ms=.5, lw=1, label=data[0][i-1])
    if i < len(data[0]):
        lastdata = data[0][i]
        lasti = i

plt.legend()
plt.show()

