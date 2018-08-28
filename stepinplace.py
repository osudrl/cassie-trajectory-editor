import pickle
import numpy

with open ("step_in_place_trajectory", 'rb') as fp:
            trajectory = pickle.load(fp)

full = numpy.array([])

for i in range(14):
	pose = numpy.copy(trajectory[i][0])
	pose = numpy.insert(pose ,0,i/14)
	n = 32 + 10+10+10
	for z in range(n):
		pose = numpy.append(pose, 0)
	print("i: %d" % i,end='')
	print("  len: %d  " % len(pose),end='')
	full = numpy.append(full,pose)

full.tofile("stepdata.bin")

