#!/usr/bin/python

from __future__ import print_function
import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
from scipy import interpolate
import sys, os

os.system('./csvik > ik.csv')


data = np.genfromtxt('ik.csv', delimiter=',', skip_header=0, skip_footer=0)
data = data.transpose()

bigindex = -1;

for i in range(len(data[1])):
    if(data[4][i] > 0.00005):
        bigindex = i

if bigindex > len(data[1]-3):
    print(-2)
else:
    print(data[1][bigindex])

# sub.plot(data[1], data[4],"-", lw=1, label='rfoot')
# sub.axhline(0.00005,  lw=1, color='black')


