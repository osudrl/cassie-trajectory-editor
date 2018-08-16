#!/usr/bin/python

from __future__ import print_function
import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import sys, os

n = 1 + 35 + 32 + 10 + 10 + 10

data = np.fromfile("stepdata.bin", dtype=np.double)
data = data.reshape((-1,n))
data = data.transpose();

d2 = np.fromfile("stepdata-2018-08-16--11-12-19.bin", dtype=np.double)
d2 = d2.reshape((-1,n))
d2 = d2.transpose();

fig = plt.figure()
sub = fig.add_subplot(111)

sub.set_xlabel('Struct Index')
sub.set_ylabel('Time Field')

sub.plot(np.arange(0,len(data[0])),data[0])
sub.plot(np.arange(0,len(d2[0])),d2[0])


plt.show()

