from robot import *
from needle_planner import *


import cisstVectorPython
from cisstVectorPython import vctFrm3

import cisstRobotPython
from cisstRobotPython import robManipulator


import numpy

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
robMap = robManipulator()

inverseFrame = frame.InverseSelf()
q1 = numpy.array([1.0,1.0,1.0,1.0,1.0,1.0,1.0])
q = numpy.array([2.0,0.0,-0.060,0.0,0.0,0.0,1.0])
a = robMap.ForwardKinematics(q)
#print frame
print robMap.InverseKinematics(q,frame)
#print inverseFrame
print a
#print q