#!/usr/bin/python

from __future__ import print_function
import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
from scipy import interpolate
import sys, os

print("15,15,93000")
print("25,25,130000")
print("10,25,130000")
print("15,025,130000")
print("15,15,93000")
print("15,5,75000")
print("20,5,75000")

print("15,1,82000")
print("15,3,80000")
print("25,3,75000")
print("50,3,65000")
print("100,3,60000")
print("200,3,50000")
print("400,3,-1")
print("400,1,65000")
print("200,10,40000")

for x in range(25, 575, 50):
    for v in range(1,10,2):
        os.system('echo %d > kcx.txt' % x)
        os.system('echo %d > kcv.txt' % v)

        print("%d,%d," % (x,v) ,end='')
        sys.stdout.flush()

        os.system('./ppp.sh')


# data = np.genfromtxt('ik.csv', delimiter=',', skip_header=0, skip_footer=0)
# data = data.transpose()

# bigindex = len(data[1]) -1;

# for i in range(len(data[1])):
#   if(data[4][i] > 0.00005):
#       bigindex = i

# print(data[1][bigindex])

# sub.plot(data[1], data[4],"-", lw=1, label='rfoot')
# sub.axhline(0.00005,  lw=1, color='black')


