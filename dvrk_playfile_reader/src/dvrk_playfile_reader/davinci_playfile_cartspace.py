from robot import *
from needle_planner import *
import csv
import IPython
import numpy as np




def moveRobotDesiredPose(robotName,filename):
	r = robot(robotName)
	gripper1_Affines = playfileReader(filename)
	
	for goalPose in range(len(gripper1_Affines)):
		print gripper1_Affines[goalPose]
		r.move_cartesian_frame(gripper1_Affines[goalPose],True)


def playfileReader(filename):
	
	fo = open(filename, "rw+")
	print "Name of the file: ", fo.name

	data=fo.readlines()
	fo.close()

	#number_lines = sum(1 for line in open(filename))
	#print number_lines

	nposes = len(data)
	print nposes

	tip_origin1 = [0,0,0]
	tip_origin2 = [0,0,0]
	x_vec1 = [0,0,0]
	y_vec1 = [0,0,0]
	z_vec1 = [0,0,0]
	gripper_ang1 = [0,0,0]

	x_vec2 = [0,0,0]
	y_vec2 = [0,0,0]
	z_vec2 = [0,0,0]
	gripper_ang2 = [0,0,0]

	x_vecs1 = []
	y_vecs1 = []
	z_vecs1 = []
	tip_origins1 = []
	gripper_angs1 = []

	x_vecs2 = []
	y_vecs2 = []
	z_vecs2 = []
	tip_origins2 = []
	gripper_angs2 = []

	arrival_times = []

	for row in csv.reader(data):
		for i in range(3):
			tip_origin1[i] = float(row[i])

			x_vec1[i] = float(row[i + 3])

			z_vec1[i] = float(row[i + 6])
			
			tip_origin2[i] = float(row[i + 10])
			
			x_vec2[i] = float(row[i + 13])

			z_vec2[i] = float(row[i + 16])

		gripper_ang1 = float(row[9])
		
		gripper_ang2 = float(row[19])

		arrival_time = float(row[20])


		x_vecs1.append(x_vec1)
		z_vecs1.append(z_vec1)
		y_vec1 = np.cross(z_vec1,x_vec1)
		y_vecs1.append(y_vec1)
		tip_origins1.append(tip_origin1[:])

		gripper_angs1.append(gripper_ang1)

		x_vecs2.append(x_vec1)
		z_vecs2.append(z_vec2)
		y_vec1 = np.cross(z_vec2,x_vec2)
		y_vecs2.append(y_vec2)
		tip_origins2.append(tip_origin2)
		gripper_angs2.append(gripper_ang2)

		arrival_times.append(arrival_time)


	N = 3
	R_numpy = np.identity(N)

	des_gripper_affine1 = Frame()
	gripper1_affines = []

	des_gripper_affine2 = Frame()
	gripper2_affines = []

	for i in range(nposes):
		R_numpy[:,0] = x_vecs1[i]
		R_numpy[:,1] = y_vecs1[i]
		R_numpy[:,2] = z_vecs1[i]

		R_numpy_FrameTemp = np.c_[R_numpy, np.ones(N)]
		R_numpy_Frame = np.r_[R_numpy_FrameTemp,[R_numpy_FrameTemp[1]]]
		R_Frame = posemath.fromMatrix(R_numpy_Frame)
		R_Frame.p = Vector(tip_origins1[i][0],tip_origins1[i][1],tip_origins1[i][2])

		gripper1_affines.append(R_Frame)

		R_numpy[:,0] = x_vecs2[i]
		R_numpy[:,1] = y_vecs2[i]
		R_numpy[:,2] = z_vecs2[i]


		R_numpy_FrameTemp = np.c_[R_numpy, np.ones(N)]
		R_numpy_Frame = np.r_[R_numpy_FrameTemp,[R_numpy_FrameTemp[1]]]
		R_Frame = posemath.fromMatrix(R_numpy_Frame)
		R_Frame.p = Vector(tip_origins2[i][0],tip_origins2[i][1],tip_origins2[i][2])

		gripper2_affines.append(R_Frame)

	return gripper1_affines


if __name__ == '__main__':
    if (len(sys.argv) != 3):
        print sys.argv[0] + ' requires two arguments, i.e. name of dVRK arm and playfile name'
    else:
        moveRobotDesiredPose(sys.argv[1],sys.argv[2])