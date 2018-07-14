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

fig = plt.figure()
# fig2 = plt.figure()
# fig2 = plt.figure()
sub = fig.add_subplot(111)
# sub2 = fig2.add_subplot(111)
# sub2 = fig2.add_subplot(111)

sub.set_xlabel('Simulation Steps')
sub.set_ylabel('Target Offet (m)')
sub.set_ylim([-0.00005,0.001])
# sub2.set_xlabel('Simulation Steps')
# sub2.set_ylabel('Target Offet (m)')

# sub2.plot(data[0], data[1],"o", ms=1, lw=0)
# sub.plot(data[1], data[2],"-", lw=1, label='pelvis')
# sub.plot(data[1], data[3],"-", lw=1, label='orientation')
sub.plot(data[1], data[4],"-", lw=1, label='rfoot')
sub.plot(data[1], data[5],"-", lw=1, label='lfoot')
# sub.plot(data[1], data[6],"-", lw=1, label='best')

# sub2.plot(data[1], data[2],"-", lw=1, label='pelvis')
# sub2.plot(data[1], data[3],"-", lw=1, label='orientation')
# sub2.plot(data[1], data[4],"-", lw=1, label='rfoot')
# sub2.plot(data[1], data[5],"-", lw=1, label='lfoot')
# sub2.plot(data[1], data[6],"-", lw=1, label='best')
sub.axhline(0,  lw=1, color='black')
sub.axhline(0.00005,  lw=1, color='black')

# boxwidth = 200

# xvals = np.arange(data[0][0],data[0][-1],boxwidth)
# minvals = [None] * len(xvals)
# for i in range(len(xvals)):
#     minvals[i] = minn(xvals[i],boxwidth,data[0],data[31])
# # sub.plot(xvals,minvals, label="mins")

# xvals = np.arange(data[0][0],data[0][-1],boxwidth)
# maxvals = [None] * len(xvals)
# for i in range(len(xvals)):
#     maxvals[i] = maxx(xvals[i],boxwidth,data[0],data[31])
# sub.plot(xvals,maxvals, label="maxs")

# maxvals = np.array(maxvals)
# minvals = np.array(minvals)

# yvals = maxvals - minvals
# sub.plot(xvals,yvals, label="diff")

# func = np.vectorize(closeToZero)
# yvals = func(yvals)
# sub.plot(xvals,yvals, label="extreme DIFF")

# xvals = data[0][0:-1]
# yvals = [None] *( len(data[0])-1)

# for i in range(len(data[0])-1):
#     yvals[i] = data[31][i+1] - data[31][i]

# sub.plot(xvals,yvals, label="derv?")


# sub.plot(xvals,yvals, label="extreme")




sub.legend()
plt.show()
# xnew = np.arange(data[0][0], data[0][-1],.0001)
# ynew = interpolate.splev(xnew, tck, der=0)

# print(tck)

# ax1.plot(xnew,ynew)

# print(ynew)

# for i in range (1,3,1):
#     tup = oof_ouch_mybones(i, data['Frame'] , data['x'])
#     ax1.plot(tup[0],tup[1],color='g')

# mymatrix = np.array([
#     [0,0,0,0,1],
#     [1,-1,1,-1,1],
#     [1,1,1,1,1],
#     [16,-8,4,-2,1],
#     [16,8,4,2,1],]) 

# invmatrix = np.linalg.inv(mymatrix)

# ans = np.array([1,2,2,-1,0])
# ans = ans.reshape(5,1);

# print(mymatrix)
# print(invmatrix)
# print (ans)

# woo = np.dot(invmatrix,ans)
# woo = woo.flatten()
# print(woo)

# poly = np.poly1d(woo)

# x_new = np.linspace(-2, 2, 100)
# y_new = poly(x_new)

# x_new += 4

# ax1.plot(x_new,y_new,color='g')


# ax1.plot(x,y, c='r', label='the data')

# leg = ax1.legend()

# for i in range(len(data['Frame'])-1):
#     m = (data['z'][i+1] - data['z'][i]) / (data['Frame'][i+1] - data['Frame'][i])
#     b = data['z'][i] - m * data['Frame'][i]
#     x = np.arange(data['Frame'][i], data['Frame'][i+1], 0.0001)
#     y = m*x + b
#     ax1.plot(x,y,color='b')

# plt.show()
# print("im a goon")

# while(True):
#     string = raw_input();
#     d = float(string)
#     print(("%.5f," % d) , end='')
#     for pose in poses:
#         print(("%.10f" % interpolate.splev(d, tcks[pose-1])) , end='')
#         if pose != poses[-1]:
#             print(",", end='')
#         else:
#             print()


