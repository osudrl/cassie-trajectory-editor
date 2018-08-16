#!/usr/bin/python

from __future__ import print_function
import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
from scipy import interpolate
import sys, os

n = 1 + 35 + 32 + 10 + 10 + 10

data = np.fromfile("stepdata.bin", dtype=np.double)
data = data.reshape((-1,n))
data = data.transpose();

fig = plt.figure()
sub = fig.add_subplot(111)

sub.set_xlabel('Simulation Steps')
sub.set_ylabel('Target Offet (m)')

print(data[0])

sub.plot(np.arange(0,len(data[0])),data[0])


plt.show()

