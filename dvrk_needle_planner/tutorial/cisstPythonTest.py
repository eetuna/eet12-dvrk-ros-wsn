from robot import *
from needle_planner import *


import cisstVectorPython
from cisstVectorPython import vctFrm3

import cisstRobotPython
from cisstRobotPython import robManipulator

import rospkg
import numpy as np
from numpy.linalg import inv
import os



from copy import *

pathTest = os.path.abspath(os.path.dirname(__file__))
filePath = pathTest + '/dvpsm.rob'
print filePath


rospack = rospkg.RosPack()

frame = vctFrm3()
inverseFrame = vctFrm3()

rotation = frame.Rotation()
rotation[0, 0] = 1.0
rotation[1, 1] = 1.0
rotation[2, 2] = 1.0
translation =  frame.Translation()
translation[0] = 2.0
translation[1] = 1.0
translation[2] = 3.0

np.set_printoptions(precision=4,suppress=True)

psm_manip = robManipulator()

result = psm_manip.LoadRobot(filePath);
print result
frame6to7 = np.matrix([[0.0, -1.0, 0.0, 0.0],
                     [0.0,  0.0, 1.0, 0.0102],
                     [-1.0, 0.0, 0.0, 0.0],
                     [0.0,  0.0, 0.0, 1.0]])


qvec = np.zeros(7)
dvrk_FK_0to6_array = psm_manip.ForwardKinematics(qvec)
dvrk_FK_0to6 = np.matrix(dvrk_FK_0to6_array)

print "Home position"
print "frame0to6: \n", frame6to7
#print type(dvrk_FK_0to6)
dvrk_FK_Test = dvrk_FK_0to6 * frame6to7
print "frame0to7: \n", dvrk_FK_Test 

#[0.0, 0.0, 0.15, 0.0, 0.0, 0.0]

#qvec_withGripper = np.array([-0.23, -0.31, 0.1, -0.14, 1.19, 1.11, 0.0])
qvec = np.array([-0.23, -0.31, 0.1, -0.14, 1.19, 1.11])
qvecIK = np.array([0., 0., 0., 0., 0., 0.])

dvrk_FK_0to6_array = psm_manip.ForwardKinematics(qvec)
dvrk_FK_0to6 = np.matrix(dvrk_FK_0to6_array)



print "FK Test"
print "using joints:", qvec
print "frame0to6 from FK: \n", dvrk_FK_0to6
#print type(dvrk_FK_0to6)
dvrk_FK_Test = dvrk_FK_0to6 * frame6to7
print "frame0to7 from FK: \n", dvrk_FK_Test 
#print type(dvrk_FK_Test)

poseGivenTest = np.matrix([
    [-0.2265,    0.4496,    0.8641,   -0.0097],
    [0.6337,    0.7417,   -0.2198,    0.0166],
    [-0.7397,    0.4978,   -0.4529,   -0.0888],
         [0,         0,         0,    1.0000]])

print "IK Test"
print "givenPose frame0to7: \n", dvrk_FK_Test
#print type(dvrk_FK_0to6)
dvrk_FK_0to6 = dvrk_FK_Test * inv(frame6to7)
print "frame0to6 from givenPose(frame0to7) using inverse of frame6to7: \n", dvrk_FK_0to6 


tolerance=1e-12;
Niteration=1000;

psm_manip.InverseKinematics(qvecIK, dvrk_FK_0to6);

#qvecIK_withGripper = np.array([qvecIK[0], qvecIK[1], qvecIK[2], qvecIK[3], qvecIK[4], qvecIK[5], 0. ])
qErr = (qvec - qvecIK)

print "original input joints: ", qvec
print "IK result joints ", qvecIK
print "IK error: ",  qErr

Jarray = np.zeros((6, 6))
Jmat = np.matrix(Jarray)
#qTest = qvec[0:-1]
#print qTest
TestJacobian = psm_manip.JacobianBody(qvec, Jmat)
print "Jacobian bool result: ", TestJacobian
print "Jacobian matrix \n", Jmat


'''

tolerance=1e-12
;Niteration=1000;

frame0to6_cisst = vctFrm3()

rotation = frame0to6_cisst.Rotation()
translation =  frame0to6_cisst.Translation()
N = 3


for i in range(N):
	for j in range(N):
		rotation[i, j] = dvrk_FK_0to6.item((i, j))

	translation[i] = dvrk_FK_0to6.item((i, j+1))


print "hop"
print dvrk_FK_0to6
'''


'''
psm_manip.InverseKinematics(frame0to6_cisst, qvecIK);

print "original input joints: ", qvec
print "IK result joints ", qvecIK
print "IK error: ",  (qvec - qvecIK)
'''

'''
inverseFrame = frame.InverseSelf()

q1 = np.array([1.0,1.0,1.0,1.0,1.0,1.0,1.0])
q = np.array([2.0,8.0,-0.060,0.0,10.0,0.0,1.0])
qtest = np.zeros(6)
q2 = np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0])


dvrk_FK_0to6 = psm_manip.ForwardKinematics(qtest)
dvrk_FK_Test = dvrk_FK_0to6 * frame6to7
#print frame
q2 = psm_manip.InverseKinematics(q,frame)
print q2
print type(dvrk_FK_Test)
#print inverseFrame
np.set_printoptions(precision=4,suppress=True)
print dvrk_FK_Test
#print '%0.2f' % dvrk_FK_Test
#print q'''