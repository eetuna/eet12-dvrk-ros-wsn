import rospy
import threading
import math
import sys
import logging
import time
import inspect
import copy
import IPython
import math

from PyKDL import *
from copy import *
from tf import transformations
from tf_conversions import posemath

from std_msgs.msg import String, Bool, Float32, Float64
from trajectory_msgs import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import JointState

from code import InteractiveConsole
from imp import new_module

import cisstVectorPython
from cisstVectorPython import vctFrm3
import cisstRobotPython
from cisstRobotPython import robManipulator

#typedef Eigen::Matrix<double, 6, 1> Vectorq6x1;
#typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;

# set some speed limits--tune these appropriately for DaVinci
q0dotmax = 0.5;
q1dotmax = 0.5;
q2dotmax = 0.5;
q3dotmax = 0.5;
q4dotmax = 1;
q5dotmax = 1;
q6dotmax = 1;
dt_traj = 0.01; # time step for trajectory interpolation
SPEED_SCALE_FACTOR= 0.5; # go this fraction of speed from above maxes

class dvrk_traj_streamer:
	def __init_(self):
		self.joint_states_ = Vector()

	def get_joint_states(self):
		return self.joint_states_
