#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Joint Trajectory Action Client Example
"""
import argparse
import sys

from copy import copy

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from dvrk_joint_trajectory_action import *

class Trajectory(object):
    def __init__(self, arm):
        ns = 'dvrk/' + arm 
        self._client = actionlib.SimpleActionClient(
            ns + "/follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )

        self.__position_joint_current = []
        self.__velocity_joint_current = []
        self.__effort_joint_current = []
        self.__robot_name = arm

        ros_namespace = '/dvrk/'
        self.__ros_namespace = ros_namespace

        full_ros_namespace = self.__ros_namespace + self.__robot_name
        rospy.Subscriber(full_ros_namespace + '/state_joint_current',
                         JointState, self.__state_joint_current_callback)

        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(arm)

    def __state_joint_current_callback(self, data):
        """Callback for the current joint position.

        :param data: the `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_current"""
        self.__position_joint_current[:] = data.position
        self.__velocity_joint_current[:] = data.velocity
        self.__effort_joint_current[:] = data.effort

    def _get_current_position(self):
        return self.__position_joint_current

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        print self._goal
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, arm):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [joint for joint in \
            ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']]


def main():
    """RSDK Joint Trajectory Example: Simple Action Client
    Creates a client of the Joint Trajectory Action Server
    to send commands of standard action type,
    control_msgs/FollowJointTrajectoryAction.
    Make sure to start the joint_trajectory_action_server.py
    first. Then run this example on a specified limb to
    command a short series of trajectory points for the arm
    to follow.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-l', '--limb', required=True, choices=['PSM1', 'PSM2'],
        help='send joint trajectory to which limb'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    arm = args.limb

    print("Initializing node... ")

    rospy.init_node("dvrk_joint_trajectory_client_%s" % (arm,))
    '''
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()'''

    print("Running. Ctrl-c to quit")
    positions = {
        'PSM1':  [0.0, 0.0, 0.001, 0, 0, 0,  0],
    }

    traj = Trajectory(arm)
    rospy.on_shutdown(traj.stop)

    current_angles = traj._get_current_position()
    traj.add_point(current_angles, 0.0)



    p1 = positions[arm]
    traj.add_point(p1, 2.0)
    traj.add_point([x * 10 for x in p1], 10.0)
    traj.add_point([x * 30 for x in p1], 30.0)
    traj.add_point([x * 60 for x in p1], 50.0)
    traj.add_point([0.001, 0.001, 0.06, 0, 0, 0,  0], 60.0)
    traj.start()
# traj.wait_for_result()
    traj.wait(65.0)
    print("Exiting - Joint Trajectory Action Test Complete")

if __name__ == "__main__":
    main()