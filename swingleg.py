#!/usr/bin/python

from __future__ import print_function
import pandas as pd
import seaborn as sns
import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
from scipy import interpolate
import sys, os


data = np.genfromtxt('swingleg.csv', delimiter=',', skip_header=0, skip_footer=0)
# data = data.transpose()

x=data[:,0]
y=data[:,1]
z=data[:,2]

heatmap = np.ones((5,5))

heatmap *= 100000

dx = max(x) - min(x)
dy = max(y) - min(y)

for i in range(len(x)):
    xx = x[i] - min(x)
    yy = y[i] - min(y)
    ix = int(int((xx/dx) * 4.99))
    iy = int(int((yy/dy) * 4.99))
    if z[i] == -1:
        heatmap[ix][iy] = 100000
    else:
        heatmap[ix][iy] = z[i]


plt.imshow(heatmap)
plt.show()

plt.show()

# sub.plot(data[1], data[4],"-", lw=1, label='rfoot')
# sub.axhline(0.00005,  lw=1, color='black')


