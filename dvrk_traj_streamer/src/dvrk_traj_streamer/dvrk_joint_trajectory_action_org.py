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
Baxter RSDK Joint Trajectory Action Server
"""
import bisect
from copy import deepcopy

import operator
import numpy as np


import rospy
import threading
import math
import sys
import logging
import time
import inspect
import IPython
import math

from PyKDL import *
from tf import transformations
from tf_conversions import posemath
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import JointState

from code import InteractiveConsole
from imp import new_module


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

#from robot import *
#from needle_planner import *

from dynamic_reconfigure.server import Server
#import baxter_control
#import baxter_dataflow
#import baxter_interface

import bezier
#import wait_for

class TestJointTrajectoryActionServer(object):
    def __init__(self, arm, reconfig_server, rate=10.0,
                 mode='position'):

        self._name = arm
        self._joint_names = {
            'PSM1': ['outer_yaw', 'outer_pitch', 'outer_insertion', 'outer_roll', 
                    'outer_wrist_pitch', 'outer_wrist_yaw', 'jaw']
                     }
        
        self._dyn = reconfig_server
        self._ns = 'dvrk/' + arm
        self._fjt_ns = self._ns + '/follow_joint_trajectory'
        self._server = actionlib.SimpleActionServer(
            self._fjt_ns,
            FollowJointTrajectoryAction,
            execute_cb=self._on_trajectory_action,
            auto_start=False)
        self._action_name = rospy.get_name()

        self._mode = mode
        if (self._mode != 'position'):
            rospy.logerr("%s: Action Server Creation Failed - "
                         "Provided Invalid Joint Control Mode '%s' (Options: "
                         "'position')" %
                    (self._action_name, self._mode,))
            return
        self._server.start()
        self._alive = True

        self._fdbk = FollowJointTrajectoryFeedback()
        self._result = FollowJointTrajectoryResult()

        # Controller parameters from arguments, messages, and dynamic
        # reconfigure
        self._control_rate = rate  # Hz
        self._control_joints = []

        self._goal_time = 0.0
        self._stopped_velocity = 0.0
        self._goal_error = dict()
        self._path_thresh = dict()

        self._coeff = [None] * len(self.joint_names())

         # Set joint state publishing to specified control rate
        self._pub_rate = rospy.Publisher(
            '/robot/joint_state_publish_rate',
             UInt16,
             queue_size=10)
        self._pub_rate.publish(self._control_rate)

        self.__robot_name = arm
        ros_namespace = '/dvrk/'
        self.__ros_namespace = ros_namespace
        self.__robot_state = 'uninitialized'
        self.__robot_state_event = threading.Event()
        self.__goal_reached = False
        self.__goal_reached_event = threading.Event()

        # continuous publish from dvrk_bridge
        self.__position_joint_desired = []
        self.__effort_joint_desired = []
        self.__position_cartesian_desired = Frame()
        self.__position_joint_current = []
        self.__velocity_joint_current = []
        self.__effort_joint_current = []
        self.__position_cartesian_current = Frame()

        full_ros_namespace = self.__ros_namespace + self.__robot_name
        self.set_robot_state_publisher = rospy.Publisher(full_ros_namespace + '/set_robot_state',
                                                         String, latch=True, queue_size = 1)
        self.set_position_joint_publisher = rospy.Publisher(full_ros_namespace + '/set_position_joint',
                                                            JointState, latch=True, queue_size = 1)
        self.set_position_goal_joint_publisher = rospy.Publisher(full_ros_namespace + '/set_position_goal_joint',
                                                                 JointState, latch=True, queue_size = 1)
        self.set_position_cartesian_publisher = rospy.Publisher(full_ros_namespace + '/set_position_cartesian',
                                                                Pose, latch=True, queue_size = 1)
        self.set_position_goal_cartesian_publisher = rospy.Publisher(full_ros_namespace + '/set_position_goal_cartesian',
                                                                     Pose, latch=True, queue_size = 1)
        self.set_jaw_position_publisher = rospy.Publisher(full_ros_namespace + '/set_jaw_position',
                                                          Float32, latch=True, queue_size = 1)
        self.set_wrench_body_publisher = rospy.Publisher(full_ros_namespace + '/set_wrench_body',
                                                         Wrench, latch=True, queue_size = 1)
        self.set_wrench_spatial_publisher = rospy.Publisher(full_ros_namespace + '/set_wrench_spatial',
                                                            Wrench, latch=True, queue_size = 1)

        # subscribers
        rospy.Subscriber(full_ros_namespace + '/robot_state',
                         String, self.__robot_state_callback)
        rospy.Subscriber(full_ros_namespace + '/goal_reached',
                         Bool, self.__goal_reached_callback)
        rospy.Subscriber(full_ros_namespace + '/state_joint_desired',
                         JointState, self.__state_joint_desired_callback)
        rospy.Subscriber(full_ros_namespace + '/position_cartesian_desired',
                         Pose, self.__position_cartesian_desired_callback)
        rospy.Subscriber(full_ros_namespace + '/state_joint_current',
                         JointState, self.__state_joint_current_callback)
        rospy.Subscriber(full_ros_namespace + '/position_cartesian_current',
                         Pose, self.__position_cartesian_current_callback)

    def wait_for(self, test, timeout=1.0, raise_on_error=True, rate=10,
             timeout_msg="timeout expired", body=None):

        """
        Waits until some condition evaluates to true.
        @param test: zero param function to be evaluated
        @param timeout: max amount of time to wait. negative/inf for indefinitely
        @param raise_on_error: raise or just return False
        @param rate: the rate at which to check
        @param timout_msg: message to supply to the timeout exception
        @param body: optional function to execute while waiting
        """
        end_time = rospy.get_time() + timeout
        rate = rospy.Rate(rate)
        notimeout = (timeout < 0.0) or timeout == float("inf")
        while not test():
            if rospy.is_shutdown():
                if raise_on_error:
                    raise OSError(errno.ESHUTDOWN, "ROS Shutdown")
                return False
            elif (not notimeout) and (rospy.get_time() >= end_time):
                if raise_on_error:
                    raise OSError(errno.ETIMEDOUT, timeout_msg)
                return False
            if callable(body):
                body()
            rate.sleep()
        return True

    def home(self):
        """This method will provide power to the robot as will as home
        the robot. This method requries the robot name."""
        rospy.loginfo(rospy.get_caller_id() + ' -> start homing')
        self.__robot_state_event.clear()
        self.set_robot_state_publisher.publish('Home')
        counter = 10 # up to 10 transitions to get ready
        while (counter > 0):
            self.__robot_state_event.wait(20) # give up to 20 secs for each transition
            if (self.__robot_state != 'DVRK_READY'):
                self.__robot_state_event.clear()
                counter = counter - 1
                rospy.loginfo(rospy.get_caller_id() + ' -> waiting for state to be DVRK_READY')
            else:
                counter = -1
        if (self.__robot_state != 'DVRK_READY'):
            rospy.logfatal(rospy.get_caller_id() + ' -> failed to reach state DVRK_READY')
        rospy.loginfo(rospy.get_caller_id() + ' <- homing complete')

    def __robot_state_callback(self, data):
        """Callback for robot state.

        :param data: the current robot state"""
        rospy.loginfo(rospy.get_caller_id() + " -> current state is %s", data.data)
        self.__robot_state = data.data
        self.__robot_state_event.set()

    def __goal_reached_callback(self, data):
        """Callback for the goal reached.

        :param data: the goal reached"""
        rospy.loginfo(rospy.get_caller_id() + " -> goal reached is %s", data.data)
        self.__goal_reached = data.data
        self.__goal_reached_event.set()

    def __state_joint_desired_callback(self, data):
        """Callback for the joint desired position.

        :param data: the `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_desired"""
        self.__position_joint_desired[:] = data.position
        self.__effort_joint_desired[:] = data.effort

    def __position_cartesian_desired_callback(self, data):
        """Callback for the cartesian desired position.

        :param data: the cartesian position desired"""
        self.__position_cartesian_desired = posemath.fromMsg(data)

    def __state_joint_current_callback(self, data):
        """Callback for the current joint position.

        :param data: the `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_current"""
        self.__position_joint_current[:] = data.position
        self.__velocity_joint_current[:] = data.velocity
        self.__effort_joint_current[:] = data.effort

    def __position_cartesian_current_callback(self, data):
        """Callback for the current cartesian position.

        :param data: The cartesian position current."""
        self.__position_cartesian_current = posemath.fromMsg(data)

    def __dvrk_set_state(self, state, timeout = 5):
        """Simple set state with block.

        :param state: the robot state
        :param timeout: the lenghth you want to wait for robot to change state
        :return: whether or not the robot state has been successfuly set
        :rtype: Bool"""
        if (self.__robot_state == state):
            return True
        self.__robot_state_event.clear()
        self.set_robot_state_publisher.publish(state)
        self.__robot_state_event.wait(timeout)
        # if the state is not changed return False
        if (self.__robot_state != state):
            rospy.logfatal(rospy.get_caller_id() + ' -> failed to reach state ' + state)
            return False
        return True

    def joint_names(self):
        """
        Return the names of the joints for the specified limb.
        @rtype: [str]
        @return: ordered list of joint names from proximal to distal
        (i.e. shoulder to wrist).
        """
        return self._joint_names[self._name]

    def get_joint_number(self):
        """Gets the number of joints on the arm specified.

        :returns: the number of joints on the specified arm
        :rtype: int"""
        joint_num = len(self.__position_joint_desired)
        return joint_num

    def __check_input_type(self, input, type_list):
        """check if the data input is a data type that is located in type_list

        :param input: The data type that needs to be checked.
        :param type_list : A list of types to check input against.
        :returns: whether or not the input is a type in type_list
        :rtype: Bool"""
        found = False
        # check the input against all input_type
        for i in range (len(type_list)):
            if (type(input) is type_list[i]):
                if(not(type(input) is list)):
                    return True
                else:
                    found = True
                    found1 = True
                    # if the list is of type list, check that each input is of
                    # the type that is after list in type_list
                    for j in range(len(input)):
                        if (not (type(input[j]) is type_list[i+1])):
                            found1 = False
                        else:
                            i+1
                    # print statements for error inside list
                    if(found1 == False):
                        print 'Error in ', inspect.stack()[1][3], 'list should be made up of', type_list[i+1],'and not of'
                        print_type1 = ' '
                        for k in range(len(input)):
                            print_medium = ' ' + str(type(input[k]))
                            print_type1 += print_medium
                        print print_type1
                    else:
                        return True
        # not of type_list print state for this error inside
        if (found == False):
            print 'Error in ', inspect.stack()[1][3], 'input is of type', input, 'and is not one of:'
            print_type2 = ''
            # skip_length
            i = 0
            while i < len(type_list):
                print_medium2 = ' '+ str(type_list[i])
                print_type2 += print_medium2
                if (type_list[i] == list):
                    i += 1
                i += 1
            print print_type2
        return False

    def __check_list_length(self, check_list, check_length):
        """check that the list is of desired length

        :param list: the list you want to check
        :param check_length: the integer to check it against
        :returns: whether or not the length of check_list is equal to check_length
        :rtype: Bool"""
        if (len(check_list) == check_length):
            return True
        else:
            print 'input is of size', len(check_list), 'but required size is', check_length
            # sperspace = new_module('superspace')
            # sperspace.check_list = check_list
            # console = Console({'superspace': superspace})
            # console.interact()
            # print 'new value of list ', superspace.check_list
            # check_list[:] = superspace.check_list
            # print 'check_list', check_list
            return False ####   should be False or actually based on user's return code from console

    def clean_shutdown(self):
        self._alive = False
        rospy.loginfo(rospy.get_caller_id() + ' -> end homing')
        self.__dvrk_set_state('DVRK_UNINITIALIZED', 20)


    def _get_trajectory_parameters(self, joint_names, goal):
        # For each input trajectory, if path, goal, or goal_time tolerances
        # provided, we will use these as opposed to reading from the
        # parameter server/dynamic reconfigure

        # Goal time tolerance - time buffer allowing goal constraints to be met
        if goal.goal_time_tolerance:
            self._goal_time = goal.goal_time_tolerance.to_sec()
        else:
            self._goal_time = self._dyn.config['goal_time']
        # Stopped velocity tolerance - max velocity at end of execution
        self._stopped_velocity = self._dyn.config['stopped_velocity_tolerance']

        # Path execution and goal tolerances per joint
        for jnt in joint_names:
            if jnt not in self.joint_names():
                rospy.logerr(
                    "%s: Trajectory Aborted - Provided Invalid Joint Name %s" %
                    (self._action_name, jnt,))
                self._result.error_code = self._result.INVALID_JOINTS
                self._server.set_aborted(self._result)
                return
            # Path execution tolerance
            path_error = self._dyn.config[jnt + '_trajectory']
            if goal.path_tolerance:
                for tolerance in goal.path_tolerance:
                    if jnt == tolerance.name:
                        if tolerance.position != 0.0:
                            self._path_thresh[jnt] = tolerance.position
                        else:
                            self._path_thresh[jnt] = path_error
            else:
                self._path_thresh[jnt] = path_error
            # Goal error tolerance
            goal_error = self._dyn.config[jnt + '_goal']
            if goal.goal_tolerance:
                for tolerance in goal.goal_tolerance:
                    if jnt == tolerance.name:
                        if tolerance.position != 0.0:
                            self._goal_error[jnt] = tolerance.position
                        else:
                            self._goal_error[jnt] = goal_error
            else:
                self._goal_error[jnt] = goal_error

    def _get_current_position(self):
        return self.__position_joint_current

    def _get_current_joint_velocity(self):
        """Gets the :ref:`current joint velocity <currentvdesired>` of the robot in terms of joint space.

        :returns: the current position of the robot in joint space
        :rtype: `JointState <http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html>`_"""
        return self.__velocity_joint_current

    def _get_current_error(self, joint_names, set_point):
        current = self._get_current_position()
        error = map(operator.sub, set_point, current)
        return zip(joint_names, error)

    def _update_feedback(self, cmd_point, jnt_names, cur_time):
        self._fdbk.header.stamp = rospy.Duration.from_sec(rospy.get_time())
        self._fdbk.joint_names = jnt_names
        self._fdbk.desired = cmd_point
        self._fdbk.desired.time_from_start = rospy.Duration.from_sec(cur_time)
        self._fdbk.actual.positions = self._get_current_position()
        self._fdbk.actual.time_from_start = rospy.Duration.from_sec(cur_time)
        self._fdbk.error.positions = map(operator.sub,
                                         self._fdbk.desired.positions,
                                         self._fdbk.actual.positions)

    def _command_stop(self, joint_angles, start_time, dimensions_dict):

        if self._mode == 'position' or self._mode == 'position_w_id':
            raw_pos_mode = (self._mode == 'position_w_id')
            if raw_pos_mode:
                pnt = JointTrajectoryPoint()
                pnt.positions = self._get_current_position()

            while (not self._server.is_new_goal_available() and self._alive):
                positions_floats = [float(np_float) for np_float in joint_angles]
                self.move_joint_list(positions_floats, interpolate = False)
                rospy.sleep(1.0 / self._control_rate)


    def move_joint_list(self, value, index = [], interpolate=True):
        """Absolute index move in joint space.

        :param value: the incremental amount in which you want to move index by, this is a list
        :param index: the incremental joint you want to move, this is a list
        :param interpolate: see  :ref:`interpolate <interpolate>`"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting abs move joint index')
        # check if value is a list
        if(self.__check_input_type(value, [list,float])):
            initial_joint_position = self.__position_joint_desired
            abs_joint = []
            abs_joint[:] = initial_joint_position
            # give index is not given and the size of the value is 7
            if (index == []):
                if(self.__check_list_length(value, len(self.__position_joint_desired))):
                    index = range(len(self.__position_joint_desired))
            # is there both an index and a value
            if(self.__check_input_type(index, [list,int]) and len(index) == len(value)):
            # if the joint specified exists
                if(len(index) <= len(initial_joint_position)):
                    for j in range(len(index)):
                        if(index[j] < len(initial_joint_position)):
                            for i in range (len(initial_joint_position)):
                                if i == index[j]:
                                    abs_joint[i] = value[j]
                    self.__move_joint(abs_joint, interpolate)

    def __move_joint(self, abs_joint, interpolate = True):
        """Absolute move by vector in joint plane.

        :param abs_joint: the absolute position of the joints in terms of a list
        :param interpolate: if false the trajectory generator will be used; if true you can bypass the trajectory generator"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting absolute move joint vector')
        if(self.__check_input_type(abs_joint, [list,float])):
            if (interpolate):
                self.__move_joint_goal(abs_joint)
            else:
                self.__move_joint_direct(abs_joint)
        rospy.loginfo(rospy.get_caller_id() + ' -> completing absolute move joint vector')

    def __move_joint_direct(self, end_joint):
        """Move the robot to the end vector by passing the trajectory generator.

        :param end_joint: the list of joints in which you should conclude movement
        :returns: true if you had succesfully move
        :rtype: Bool"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting move joint direct')
        if (self.__check_input_type(end_joint, [list,float])):
            if not self.__dvrk_set_state('DVRK_POSITION_JOINT'):
                return False
            # go to that position directly
            joint_state = JointState()
            joint_state.position[:] = end_joint
            print joint_state
            self.set_position_joint_publisher.publish(joint_state)
            rospy.loginfo(rospy.get_caller_id() + ' <- completing move joint direct')
            return True

    def __move_joint_goal(self, end_joint):
        """Move the robot to the end vector by bypassing the trajectory generator.

        :param end_joint: the list of joints in which you should conclude movement
        :returns: true if you had succesfully move
        :rtype: Bool"""
        rospy.loginfo(rospy.get_caller_id() + ' -> starting move joint goal')
        if (self.__check_input_type(end_joint, [list,float])):
            if (not self.__dvrk_set_state('DVRK_POSITION_GOAL_JOINT')):
                return False
            joint_state = JointState()
            joint_state.position[:] = end_joint
            self.__set_position_goal_joint_publish_and_wait(joint_state)
            return True

    def __set_position_goal_joint_publish_and_wait(self, end_position):
        """Wrapper around publisher/subscriber to manage events for joint coordinates.

        :param end_position: there is only one parameter, end_position which tells us what the ending position is
        :returns: whether or not you have successfully moved by goal or not
        :rtype: Bool"""
        self.__goal_reached_event.clear()
        self.__goal_reached = False
        self.set_position_goal_joint_publisher.publish(end_position)
        self.__goal_reached_event.wait(20) # 1 minute at most
        if not self.__goal_reached:
            return False
        rospy.loginfo(rospy.get_caller_id() + ' -> completing set position goal joint publish and wait')
        return True

    def _command_joints(self, joint_names, point, start_time, dimensions_dict):

        if self._server.is_preempt_requested():
            rospy.loginfo("%s: Trajectory Preempted" % (self._action_name,))
            self._server.set_preempted()
            self._command_stop(self._get_current_position(), start_time, dimensions_dict)
            return False
        deltas = self._get_current_error(joint_names, point.positions)

        for delta in deltas:
            if ((math.fabs(delta[1]) >= self._path_thresh[delta[0]]
                and self._path_thresh[delta[0]] >= 0.0)):
                rospy.logerr("%s: Exceeded Error Threshold on %s: %s" %
                             (self._action_name, delta[0], str(delta[1]),))
                self._result.error_code = self._result.PATH_TOLERANCE_VIOLATED
                self._server.set_aborted(self._result)
                self._command_stop(self._get_current_position(), start_time, dimensions_dict)
                return False
        if ((self._mode == 'position' or self._mode == 'position_w_id')
              and self._alive):
            cmd = dict(zip(joint_names, point.positions))
            raw_pos_mode = (self._mode == 'position_w_id')

            positions_floats = [float(np_float) for np_float in point.positions]
            self.move_joint_list(positions_floats, interpolate = False)
        return True

    def _get_bezier_point(self, b_matrix, idx, t, cmd_time, dimensions_dict):
        pnt = JointTrajectoryPoint()
        pnt.time_from_start = rospy.Duration(cmd_time)
        num_joints = b_matrix.shape[0]
        pnt.positions = [0.0] * num_joints
        for jnt in range(num_joints):
            b_point = bezier.bezier_point(b_matrix[jnt, :, :, :], idx, t)
            # Positions at specified time
            pnt.positions[jnt] = b_point[0]
        return pnt

    def _compute_bezier_coeff(self, trajectory_points, dimensions_dict):
        # Compute Full Bezier Curve
        num_joints = self.get_joint_number()
        num_traj_pts = len(trajectory_points)
        num_traj_dim = sum(dimensions_dict.values())
        num_b_values = len(['b0', 'b1', 'b2', 'b3'])
        b_matrix = np.zeros(shape=(num_joints, num_traj_dim, num_traj_pts-1, num_b_values))
        for jnt in xrange(num_joints):
            traj_array = np.zeros(shape=(len(trajectory_points), num_traj_dim))
            for idx, point in enumerate(trajectory_points):
                current_point = list()
                current_point.append(point.positions[jnt])
                traj_array[idx, :] = current_point
            d_pts = bezier.de_boor_control_pts(traj_array)
            b_matrix[jnt, :, :, :] = bezier.bezier_coefficients(traj_array, d_pts)
        return b_matrix

    def _determine_dimensions(self, trajectory_points):
        # Determine dimensions supplied
        position_flag = True
        return {'positions':position_flag}

    def _on_trajectory_action(self, goal):
        #print goal.trajectory
        joint_names = goal.trajectory.joint_names
        trajectory_points = goal.trajectory.points
        # Load parameters for trajectory
        self._get_trajectory_parameters(joint_names, goal)
        # Create a new discretized joint trajectory
        num_points = len(trajectory_points)
        if num_points == 0:
            rospy.logerr("%s: Empty Trajectory" % (self._action_name,))
            self._server.set_aborted()
            return
        rospy.loginfo("%s: Executing requested joint trajectory" %
                      (self._action_name,))
        rospy.logdebug("Trajectory Points: {0}".format(trajectory_points))
        control_rate = rospy.Rate(self._control_rate)

        dimensions_dict = self._determine_dimensions(trajectory_points)

        if num_points == 1:
            # Add current position as trajectory point
            first_trajectory_point = JointTrajectoryPoint()
            first_trajectory_point.positions = self._get_current_position()
            # To preserve desired velocities and accelerations, copy them to the first
            # trajectory point if the trajectory is only 1 point.
            first_trajectory_point.time_from_start = rospy.Duration(0)
            trajectory_points.insert(0, first_trajectory_point)
            num_points = len(trajectory_points)

        # Compute Full Bezier Curve Coefficients for all 7 joints
        pnt_times = [pnt.time_from_start.to_sec() for pnt in trajectory_points]
        try:
            b_matrix = self._compute_bezier_coeff(
                                                  trajectory_points,
                                                  dimensions_dict)
        except Exception as ex:
            rospy.logerr(("{0}: Failed to compute a Bezier trajectory for {1}"
                         " arm with error \"{2}: {3}\"").format(
                                                  self._action_name,
                                                  self._name,
                                                  type(ex).__name__, ex))
            self._server.set_aborted()
            return
        # Wait for the specified execution time, if not provided use now
        start_time = goal.trajectory.header.stamp.to_sec()
        #print start_time
        #print rospy.get_time()

        if start_time == 0.0:
            start_time = rospy.get_time()
        
        self.wait_for(
            lambda: rospy.get_time() >= start_time,
            timeout=float('inf')
        )
        # Loop until end of trajectory time.  Provide a single time step
        # of the control rate past the end to ensure we get to the end.
        # Keep track of current indices for spline segment generation
        now_from_start = rospy.get_time() - start_time
        end_time = trajectory_points[-1].time_from_start.to_sec()
        while (now_from_start < end_time and not rospy.is_shutdown()):
            #Acquire Mutex
            now = rospy.get_time()
            now_from_start = now - start_time
            idx = bisect.bisect(pnt_times, now_from_start)
            #Calculate percentage of time passed in this interval
            if idx >= num_points:
                cmd_time = now_from_start - pnt_times[-1]
                t = 1.0
            elif idx >= 0:
                cmd_time = (now_from_start - pnt_times[idx-1])
                t = cmd_time / (pnt_times[idx] - pnt_times[idx-1])
            else:
                cmd_time = 0
                t = 0

        point = self._get_bezier_point(b_matrix, idx,
                                           t, cmd_time,
                           dimensions_dict)

        # Command Joint Position, Velocity, Acceleration
        command_executed = self._command_joints(joint_names, point, start_time, dimensions_dict)
        self._update_feedback(deepcopy(point), joint_names, now_from_start)
        # Release the Mutex
        if not command_executed:
            return
        control_rate.sleep()
        # Keep trying to meet goal until goal_time constraint expired
        last = trajectory_points[-1]
        last_time = trajectory_points[-1].time_from_start.to_sec()
        end_angles = last.positions

        def check_goal_state():
            for error in self._get_current_error(joint_names, last.positions):
                if (self._goal_error[error[0]] > 0
                        and self._goal_error[error[0]] < math.fabs(error[1])):
                    return error[0]
            if (self._stopped_velocity > 0.0 and
                max([abs(cur_vel) for cur_vel in self._get_current_joint_velocity()]) >
                    self._stopped_velocity):
                return False
            else:
                return True

        while (now_from_start < (last_time + self._goal_time)
               and not rospy.is_shutdown()):
            if not self._command_joints(joint_names, last, start_time, dimensions_dict):
                return
            now_from_start = rospy.get_time() - start_time
            self._update_feedback(deepcopy(last), joint_names, 
                                  now_from_start)
            control_rate.sleep()

        now_from_start = rospy.get_time() - start_time
        self._update_feedback(deepcopy(last), joint_names, 
                                  now_from_start)

        # Verify goal constraint
        result = check_goal_state()
        if result is True:
            rospy.loginfo("%s: Joint Trajectory Action Succeeded for %s arm" %
                          (self._action_name, self._name))
            self._result.error_code = self._result.SUCCESSFUL
            self._server.set_succeeded(self._result)
        elif result is False:
            rospy.logerr("%s: Exceeded Max Goal Velocity Threshold for %s arm" %
                         (self._action_name, self._name))
            self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
            self._server.set_aborted(self._result)
        else:
            rospy.logerr("%s: Exceeded Goal Threshold Error %s for %s arm" %
                         (self._action_name, result, self._name))
            self._result.error_code = self._result.GOAL_TOLERANCE_VIOLATED
            self._server.set_aborted(self._result)
        self._command_stop(end_angles, start_time, dimensions_dict)
