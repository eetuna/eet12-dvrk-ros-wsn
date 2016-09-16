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

from robot import *
from needle_planner import *

import actionlib
from basics.msg import TrajAction, TrajGoal, TrajResult, TrajFeedback

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)

def execute(goal):

