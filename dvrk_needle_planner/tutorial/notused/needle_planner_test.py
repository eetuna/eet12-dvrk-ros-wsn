from robot import *
from needle_planner import *
import PyKDL

#create a square of length `length`
def needle_planner_test():
    n = needle_planner()
    print n.needle_radius_
    print n.needle_axis_ht_

def checkTransf():
	v = PyKDL.Vector(1,3,5)

	# create a rotation from Roll Pitch, Yaw angles
	r1 = PyKDL.Rotation.RPY(1.2, 3.4, 0)

	# create a rotation from XYZ Euler angles
	r2 = PyKDL.Rotation.EulerZYX(0, 1, 0)

	# create a rotation from a rotation matrix
	r3 = PyKDL.Rotation(1,0,0, 0,1,0, 0,0,1)

	# create a frame from a vector and a rotation
	frame1 = PyKDL.Frame(r2, v)
	print frame1

if __name__ == '__main__':
    needle_planner_test() 
    #checkTransf()