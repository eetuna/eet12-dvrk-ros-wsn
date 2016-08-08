"""@package docstring
Documentation for this test file

More details:

This file tests the cisst python wrappers for cisstVector and cisstRobot classes.

It generates an instance of vctFrm3, assigns its rotation and translation counterparts
It loads the dvrk psm kinematics, which is defined as a .rob file. It gets the file path and pass thi string.
Creates an instance of a robManipulator
Defines the transformation from dvrk 6th frame to gripper frame
Tests the FK for a predefined set of joint values: T_6/0 = FK(qvec)
Do not append the gripper jaw joint. Only include the first 6 joint as the FK and IK of cisstRobot class based on 6 joints
To get the end effector, first find the pose of the 6th frame w.r.t base from FK. Then multiply this pose with the transformation of T_6/0 * T_7/6 = T_7/0
From the computed pose T_7/0 get the T_6/0 by T_6/0 = T_7/0*inv(T_6/7)
The compute the joint angles by IK(T_6/0) = qvec_IK
Compare qvec_IK with the original qvec and find the error: qerr = qvec - qvec_IK
Set the precision of np.array and np.matrix by np.set_printoptions. Suppress the scientific, i.e. e+00, notation
Calculate the BodyJacobian and pass the computed value to a matrix Jmat.

"""



import cisstVectorPython
from cisstVectorPython import vctFrm3
import cisstRobotPython
from cisstRobotPython import robManipulator

import rospkg
import os

import numpy as np
from numpy.linalg import inv

from robot import *
from needle_planner import *
#from copy import *

""" get the path of the kinematics .rob file """
pathTest = os.path.abspath(os.path.dirname(__file__))
filePath = pathTest + '/dvpsm.rob'
print filePath

""" Generate an instance of the robManipulator class """
""" Load the robot kinematics file by passing the .rob path"""
psm_manip = robManipulator()
result = psm_manip.LoadRobot(filePath);
#print result

""" cisstVector test """
""" generate an empty transformation, initialize the rotation and translation """
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

""" set the print options of np type matrices and arrays """
np.set_printoptions(precision=4,suppress=True)


""" Define the transformation from 6th frame to gripper """
frame6to7 = np.matrix([[0.0, -1.0, 0.0, 0.0],
                     [0.0,  0.0, 1.0, 0.0102],
                     [-1.0, 0.0, 0.0, 0.0],
                     [0.0,  0.0, 0.0, 1.0]])


""" Find the home pose of dvrk psm """
""" calculate the Home pose using FK with all joints values set to 0.0 """
""" Do not include the gripper for the FK and IK as the robManipulator only for the first 6 joints"""
qvec = np.zeros(7)
dvrk_FK_0to6_array = psm_manip.ForwardKinematics(qvec)
dvrk_FK_0to6 = np.matrix(dvrk_FK_0to6_array)
dvrk_FK_Test = dvrk_FK_0to6 * frame6to7

""" Print the Home position """
print "\n Home position \n"
print "frame0to6: \n", frame6to7
#print type(dvrk_FK_0to6)
print "frame0to7: \n", dvrk_FK_Test 

#[0.0, 0.0, 0.15, 0.0, 0.0, 0.0]
#qvec_withGripper = np.array([-0.23, -0.31, 0.1, -0.14, 1.19, 1.11, 0.0])

""" Define a valid joint configuration, qvec """
qvec = np.array([-0.23, -0.31, 0.1, -0.14, 1.19, 1.11])
""" Initialize the IK output joint vector """
qvecIK = np.array([0., 0., 0., 0., 0., 0.])

""" Find the corresponding pose from FK for this joint configuration """
dvrk_FK_0to6_array = psm_manip.ForwardKinematics(qvec)
dvrk_FK_0to6 = np.matrix(dvrk_FK_0to6_array)
dvrk_FK_Test = dvrk_FK_0to6 * frame6to7

""" print the pose """
print "\n FK Test \n"
print "using joints:", qvec
print "frame0to6 from FK: \n", dvrk_FK_0to6
print "frame0to7 from FK: \n", dvrk_FK_Test 
#print type(dvrk_FK_Test)

poseGivenTest = np.matrix([
    [-0.2265,    0.4496,    0.8641,   -0.0097],
    [0.6337,    0.7417,   -0.2198,    0.0166],
    [-0.7397,    0.4978,   -0.4529,   -0.0888],
         [0,         0,         0,    1.0000]])


""" Do the IK test """
""" 
From the computed pose in FK test step 
compute the pose w.r.t. base frame using the inv(T_7/6)
Find the joint configuration for this pose using IK, qvec_IK
Compare qvec_IK with qvec and find qerr = qvec - qvec_IK
"""

dvrk_FK_0to6 = dvrk_FK_Test * inv(frame6to7)
tolerance=1e-12
Niteration=1000
psm_manip.InverseKinematics(qvecIK, dvrk_FK_0to6)
qErr = (qvec - qvecIK)

""" print the gripper and 6th frame w.r.t to base """
print "\n IK Test \n"
print "givenPose frame0to7: \n", dvrk_FK_Test
#print type(dvrk_FK_0to6)
print "frame0to6 from givenPose(frame0to7) using inverse of frame6to7: \n", dvrk_FK_0to6 

""" print qvec, qvecIK, and qerr """
#qvecIK_withGripper = np.array([qvecIK[0], qvecIK[1], qvecIK[2], qvecIK[3], qvecIK[4], qvecIK[5], 0. ])
print "original input joints: ", qvec
print "IK result joints ", qvecIK
print "IK error: ",  qErr

""" Test the Body Jacobian """
""" A 6x6 Jacoban matrix, Jmat should be passed as an input parameter """ 
Jarray = np.zeros((6, 6))
Jmat = np.matrix(Jarray)
#qTest = qvec[0:-1]
#print qTest
TestJacobian = psm_manip.JacobianBody(qvec, Jmat)

""" print the computed Jacobian matrix """
print "\n Jacobian Computation \n"
print "Jacobian bool result: ", TestJacobian
print "Jacobian matrix \n", Jmat


