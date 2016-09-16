from robot import *
import PyKDL

#create a square of length `length`
def square(robotName):
    r = robot(robotName)
    
    r.move_cartesian_translation([0.0,0.0,-0.15])
    #the length of the square
    input_length = raw_input('Enter the length of the square : ')
    length = float(input_length)

    #move in cartesian space
    r.move_cartesian_translation([0.0,length, -0.15])
    r.move_cartesian_translation([length, length, -0.15])
    r.move_cartesian([length, 0.0, -0.15])
    r.move_cartesian([0.0, 0.0, -0.15])

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
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        square(sys.argv[1])
        checkTransf()