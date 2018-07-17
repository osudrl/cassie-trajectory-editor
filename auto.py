#!/usr/bin/python

from __future__ import print_function
import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
from scipy import interpolate
import sys, os

xrangee = [.1,1,2,3,4,5,6,7,8,9,10,15,20,25,50,75,100,125,150,175,200,225,275,325,375,425,475,525,575,625,675,725,775,825,875,925,975,1025,1250,1500,1750,2000,2500,3000,4000,5000,6000,7000,8000,9000,10000,12000,15000,18000,22500,50000,75000,100000]
vrange = [0.001, 0.01, 0.1, 1, 2, 3, 5, 7, 9, 12, 15, 19, 22, 25, 30, 35, 42, 50, 62, 75, 92, 100, 125, 150, 175, 225, 300,350,425,550,750,1000]

for x in xrangee:
    for v in vrange:
        os.system('echo %.3f > kcx.txt' % x)
        os.system('echo %.3f > kcv.txt' % v)

        print("%.3f,%.3f," % (x,v) ,end='')
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


