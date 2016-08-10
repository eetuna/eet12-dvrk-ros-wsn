"""
 * File:   needle_plan_horiz_kvec.cpp
 * Author: wsn
 *
 * Created March 19, 2016, wsn
 * This version: listens for a geometry_msgs/Polygon message on topic: entrance_and_exit_pts
 * ASSUMES the tissue is horizontal; constructs corresponding kvec_yaw angle and needle center point
 * Computes a corresponding cps file for gripper1 motion for circular needle drive
 * saves this file to disk as: 
 """

import rospy
import threading
import math
import sys
import logging
import time
import inspect
import code
import IPython
import math
#import PyKDL

import numpy as np

from PyKDL import *
from copy import *
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

from robot import *
from needle_planner import *

#include <needle_planner/needle_planner.h>
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Vector3.h>

g_affine_lcamera_to_psm_one = Frame()
g_affine_lcamera_to_psm_two = Frame()
g_psm1_start_pose = Frame()
g_psm2_start_pose = Frame()

g_O_entry_point = np.array([0.0, 0.0, 0.0])
g_O_exit_point = np.array([0.0, 0.0, 0.0])

gripper_affines_wrt_camera = []
gripper_affines_wrt_psm = []
psm2_gripper_affines_wrt_camera = []
psm2_gripper_affines_psm = []

""" set the print options of np type matrices and arrays """
np.set_printoptions(precision=6,suppress=True)

def init_poses():
    rospy.loginfo("getting transforms from camera to PSMs")
    tferr = True
    ntries = 0
    rospy.loginfo("waiting for tf between base and camera...")


    if tferr:
        g_affine_lcamera_to_psm_one.p = Vector(-0.155,-0.03265,0.0)
        g_affine_lcamera_to_psm_two.p = Vector(0.145, -0.03265, 0.0)

        g_affine_lcamera_to_psm_one.M = Rotation(-1,0,0, 0,1,0, 0,0,-1)
        g_affine_lcamera_to_psm_two.M = Rotation(-1,0,0, 0,1,0, 0,0,-1)
        rospy.logwarn("using default transform")
    else:
        rospy.loginfo("tf is good")

    rospy.loginfo("transofrm from left camera to psm one:")
    print g_affine_lcamera_to_psm_one
    rospy.loginfo("transofrm from left camera to psm two:")
    print g_affine_lcamera_to_psm_two

    #default start pose, if can't get tf
    if tferr:
        g_psm1_start_pose.p = Vector(-0.02, 0, 0.04)
        g_psm2_start_pose.p = Vector(0.02, 0, 0.04)

        g_psm1_start_pose.M = Rotation(-1,0,0, 0,1,0, 0,0,-1)
        g_psm2_start_pose.M = Rotation(-1,0,0, 0,1,0, 0,0,-1)
        rospy.logwarn("using default start poses")
    else:
        rospy.loginfo("tf is good")

    rospy.loginfo("psm1 gripper start pose:")
    print g_psm1_start_pose
    rospy.loginfo("psm2 gripper start pose:")
    print g_psm2_start_pose

    global g_O_entry_point
    global g_O_exit_point

    g_O_entry_point[0] = 0.0
    g_O_entry_point[1] = 0.0
    g_O_entry_point[2] = 0.12
    g_O_exit_point = deepcopy(g_O_entry_point)
    g_O_exit_point[0] = g_O_exit_point[0] + 0.02


if __name__ == '__main__':

    #ROS sets-up:
    rospy.init_node('needle_planner_test_main') #node name
    
    init_poses()
    print g_O_exit_point
    print g_O_entry_point

    rospy.loginfo("main: instantiating an object of type needle_planner")
    needlePlanner = needle_planner()

    needlePlanner.set_affine_lcamera_to_psm_one(g_affine_lcamera_to_psm_one);
    needlePlanner.set_affine_lcamera_to_psm_two(g_affine_lcamera_to_psm_two);


    print needlePlanner.default_affine_lcamera_to_psm_one_
    print needlePlanner.default_affine_lcamera_to_psm_two_


    r_needle = DEFAULT_NEEDLE_RADIUS
    print r_needle


    rospy.loginfo("entering loop")
    count = 0
    while not rospy.is_shutdown():
        if (count == 1):
            print "count: ",count
            break
        #compute O_needle from entry and exit points:
        O_needle = 0.5 * (g_O_entry_point + g_O_exit_point)
        O_needle[2] -= DEFAULT_NEEDLE_AXIS_HT

        in_to_out_vec = g_O_exit_point - g_O_entry_point
        #vector from entry to exit is 90-deg away from needle z-axis, so add pi/2
        #print "atan2 result: ",math.atan2(in_to_out_vec[1], in_to_out_vec[0])
        #print in_to_out_vec[1]
        #print in_to_out_vec[0]
        #print in_to_out_vec
        kvec_yaw = math.atan2(in_to_out_vec[1], in_to_out_vec[0]) + math.pi/2.0
        rospy.loginfo("using kvec_yaw = %f"%kvec_yaw)


        print "O_needle", O_needle
        print "r_needle", r_needle
        #print "gripper_affines_wrt_camera", gripper_affines_wrt_camera
        #compute the tissue frame in camera coords, based on point-cloud selections:
        print "hop", gripper_affines_wrt_camera
        print "hop2", gripper_affines_wrt_psm
        needlePlanner.simple_horiz_kvec_motion(O_needle, r_needle, kvec_yaw, gripper_affines_wrt_camera, gripper_affines_wrt_psm)

        nposes = len(gripper_affines_wrt_camera)
        print "nposes: ",nposes
        rospy.logwarn("computed %d gripper poses w/rt camera"%nposes)


        print 
        for i in range(0,nposes):
            rospy.loginfo("pose %d"%i)
            print "camera\n"
            print "Rotation:", gripper_affines_wrt_camera[i].M
            print "origin:", gripper_affines_wrt_camera[i].p
            print "gripper\n"
            print "Rotation:", gripper_affines_wrt_psm[i].M
            print "origin:", gripper_affines_wrt_psm[i].p


        needlePlanner.write_needle_drive_affines_to_file(gripper_affines_wrt_camera, gripper_affines_wrt_psm)


        #d = rospy.Duration(0.01, 0)
        rospy.sleep(0.01)
        count += 1