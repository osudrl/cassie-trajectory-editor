#!/usr/bin/python

import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
import numpy as np

# data = np.genfromtxt('tens.csv', delimiter=',', skip_header=1,
#                      skip_footer=1, names=['Frame', 'x', 'y', 'z'])
data = np.genfromtxt('points.csv', delimiter=',', skip_header=0,
                     skip_footer=0, names=['Frame', 'x'])

fig = plt.figure()

ax1 = fig.add_subplot(111)


ax1.set_title("Interpolation?")    
ax1.set_xlabel('time')
ax1.set_ylabel('Mains voltage')
ax1.scatter(data['Frame'], data['x'], s=10, color='r', label='the data')


mymatrix = np.array([
    [0,0,0,0,1],
    [1,-1,1,-1,1],
    [1,1,1,1,1],
    [16,-8,4,-2,1],
    [16,8,4,2,1],]) 

invmatrix = np.linalg.inv(mymatrix)

ans = np.array([1,2,2,-1,0])
ans = ans.reshape(5,1);

print(mymatrix)
print(invmatrix)
print (ans)

woo = np.dot(invmatrix,ans)
woo = woo.flatten()
print(woo)

poly = np.poly1d(woo)

x_new = np.linspace(-2, 2, 100)
y_new = poly(x_new)

x_new += 4

ax1.plot(x_new,y_new,color='g')


# ax1.plot(x,y, c='r', label='the data')

leg = ax1.legend()

for i in range(len(data['Frame'])-1):
    m = (data['x'][i+1] - data['x'][i]) / (data['Frame'][i+1] - data['Frame'][i])
    b = data['x'][i] - m * data['Frame'][i]
    x = np.arange(data['Frame'][i], data['Frame'][i+1], 0.0001)
    y = m*x + b
    ax1.plot(x,y,color='b')

plt.show()

