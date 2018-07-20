#!/usr/bin/python

from __future__ import print_function
import matplotlib as mpl
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cbook as cbook
from scipy import interpolate
import sys, os

import subprocess
p = subprocess.Popen(['bash', 'statfool.sh'], stdout=subprocess.PIPE,stderr=subprocess.PIPE)
out, err = p.communicate()
nbytes = int(out)

n = 1 + 1 + 35 + 35
count = (nbytes/8) - ((nbytes/8)%n)
data = np.fromfile("fool.bin", dtype=np.double, count=count)
data = data.reshape((-1,n))

frames = data[:, 0]
steps = data[:, 1]
initq = data[:, 2:2+35].transpose()
ikq = data[:, 2+35:2+35+35].transpose()

fig = plt.figure()
sub = fig.add_subplot(111)

sub.set_xlabel('Frame Offset')
sub.set_ylabel('Qpos Value')
# sub.set_ylim([-0.00005,0.001])




# # sub.plot(frames, initq[21],"o", ms=1, label='old_r_roll')
# sub.plot(frames, initq[22],"o", ms=1, label='old_r_pitch')
# sub.plot(frames, initq[23],"o", ms=1, label='old_r_yaw')
# sub.plot(frames, initq[28],"o", ms=1, label='old_r_knee')
# # sub.plot(frames, initq[29],"o", ms=1, label='old_r_shin')
# sub.plot(frames, initq[30],"o", ms=1, label='old_r_tarsus')
# sub.plot(frames, initq[34],"o", ms=1, label='old_r_foot34')

sub.plot(frames, steps/500, "o", ms=2, label='steps/500')

frames = np.concatenate((frames,frames))
ikq = np.concatenate((initq, ikq), axis=1)

# sub.plot(frames, ikq[21],"o", ms=1, label='both_r_roll')
sub.plot(frames, ikq[22],"o", ms=1, label='both_r_pitch')
sub.plot(frames, ikq[23],"o", ms=1, label='both_r_yaw')
sub.plot(frames, ikq[28],"o", ms=1, label='both_r_knee')
# sub.plot(frames, ikq[29],"o", ms=1, label='both_r_shin')
sub.plot(frames, ikq[30],"o", ms=1, label='both_r_tarsus')
sub.plot(frames, ikq[34],"o", ms=1, label='both_r_foot34')

sub.legend()
plt.show()

