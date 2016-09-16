import bisect
from copy import deepcopy
import math
import operator
import numpy as np


import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryFeedback,
    FollowJointTrajectoryResult,
)
from std_msgs.msg import (
    UInt16,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from dynamic_reconfigure.server import Server

from robot import *
from needle_planner import *
from dvrk_joint_trajectory_action import *

from dvrk_traj_streamer.cfg import (
    PositionJointTrajectoryActionServerConfig )

#from config import Config

import argparse

#rospy.init_node("dvrk_joint_trajectory_action_server")



""
arm = 'PSM1'
rate = 100
mode = 'position'
global r1
r1 = robot('PSM1')

dyn_cfg_srv = Server(PositionJointTrajectoryActionServerConfig,
                     lambda config, level: config)

j = TestJointTrajectoryActionServer(r, arm, dyn_cfg_srv, rate, mode, )

set_point = [0, 0 ,0.05, 0, 0, 0, 0]
print j._get_current_position()

print j._get_current_error(set_point)
#arm2 = 'PSM2'
#r = robot(arm2)
#print r.get_current_joint_position()