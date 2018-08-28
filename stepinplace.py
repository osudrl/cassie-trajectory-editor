import pickle
import numpy

with open ("step_in_place_trajectory", 'rb') as fp:
            trajectory = pickle.load(fp)

for i in range(28):
	pose = numpy.copy(trajectory[i][0])
	print("i: %d" % i,end='')
	print("  len: %d  " % len(pose),end='')
	print(pose)


