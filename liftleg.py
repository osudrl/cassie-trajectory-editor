#!/usr/bin/python

from __future__ import print_function
import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
from scipy import interpolate
import sys, os



# data = np.genfromtxt('auto-liftleg.csv', delimiter=',', skip_header=0, skip_footer=0)
data = np.genfromtxt('auto-swingleg.csv', delimiter=',', skip_header=0, skip_footer=0)
data = data.transpose()

fig = plt.figure(figsize=(8, 6), dpi=100)
sub = fig.add_subplot(111)

for i in range(len(data[2])):
    if data[2][i] < 0:
        data[2][i] = 160000

lasti = 0
i = 0
lastdata = None


while i < len(data[0]):
    while i < len(data[0]) and  data[0][i] == lastdata:
        i += 1
    if i < len(data[0]):
        lastdata = data[0][i]
        if(data[0][i] > 300 and data[0][i] < 15000) :
            sub.plot(data[1][lasti:i], data[2][lasti:i],'-o' , ms=.5, lw=.5, label=('x=%.1f' % data[0][i]))
        lasti = i



# while i < len(data[1]):
#     while i < len(data[1]) and  data[1][i] == lastdata:
#         i += 1
#     if i < len(data[1]):
#         lastdata = data[1][i]
#         sub.plot(data[0][lasti:i], data[2][lasti:i],'-o' , ms=.5, lw=2, label=('x=%.1f' % data[1][i]))
#         lasti = i

sub.legend()
plt.show()
