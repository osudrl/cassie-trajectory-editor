#!/usr/bin/python

import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
import numpy as np

data = np.genfromtxt('out-traj.csv', delimiter=',', skip_header=1,
                     skip_footer=1, names=['Frame', 'x', 'y', 'z'])

fig = plt.figure()

ax1 = fig.add_subplot(111)


ax1.set_title("Mains power stability")    
ax1.set_xlabel('time')
ax1.set_ylabel('Mains voltage')
ax1.scatter(data['Frame'], data['z'], s=1, color='r', label='the data')

# ax1.plot(x,y, c='r', label='the data')

leg = ax1.legend()

for i in range(len(data['Frame'])-1):
    m = (data['z'][i+1] - data['z'][i]) / (data['Frame'][i+1] - data['Frame'][i])
    b = data['z'][i] - m * data['Frame'][i]
    x = np.arange(data['Frame'][i], data['Frame'][i+1], 0.001)
    y = m*x + b
    ax1.plot(x,y,color='b')

plt.show()

