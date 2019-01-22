from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt 
import os
import numpy as np

def decodeCalFile(s):
    output = []
    output_angle = []

    split = s.split("\t")

    for sq in split:
        if '[' in sq:
            o = sq.replace("[","")
            o = o.replace("]","")
            sq = o.split()
            ret = []
            for num in sq:
                ret.append(float(num))
            output.append(ret)
        elif sq == '\n':
        	pass
        else:
            try:
                output_angle.append(float(sq))
            except:
                print("***Error Input***", sq)
                print()
    return np.array(output).reshape(10,10,3), np.array(output_angle)

def plot3D(split_array):
	plt.ion()
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')

	ax.scatter(split_array[:,:,0], split_array[:,:,1], split_array[:,:,2], c='r', marker='o')

	ax.set_xlabel('X Label')
	ax.set_ylabel('Y Label')
	ax.set_zlabel('Z Label')

	ax.view_init(elev=-102, azim=-90)

	plt.xlim(-1, 1) 
	plt.ylim(-1, 1) 
	ax.set_zlim(2, 10) 

	plt.show()


f = open(os.path.dirname(os.path.realpath(__file__)) +"/" + "CalFile3.txt", "r")
s = f.readline()

while s != []:
	split_array, angle_map = decodeCalFile(s)
	plot3D(split_array)
	inp = raw_input("Please press anything to continue")
	plt.close()
	s = f.readline()


