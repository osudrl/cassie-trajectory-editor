#!/usr/bin/python

from __future__ import print_function
import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
from scipy import interpolate

data = np.genfromtxt('out-traj.csv', delimiter=',', skip_header=1, skip_footer=1)

data = data.transpose()

fig = plt.figure()
sub = fig.add_subplot(111)

xvals = np.arange(data[0][0],data[0][-1],1)
poses = np.arange(1,len(data),1)
for pose in poses:
    poly = np.polyfit(data[0], data[pose],11)
    poly = np.poly1d(poly)
    yvals = poly(xvals)
    sub.plot(xvals,yvals, label=("%d"%pose))
    sub.plot(data[0],data[pose], "ro",linewidth=0, ms=1)

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


