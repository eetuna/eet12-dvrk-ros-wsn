/* 
 * File:   needle_planner_kvec_horiz_test_main_v2.cpp
 * Author: wsn
 *
 * Created Mar 17, 2016
 * this version: test needle drive for various kvec angles
 * Extend: accept needle entry point; compute needle drives for pivots about entry point
 *  need to specify height of needle center above tissue
 */

from robot import *
from needle_planner import *
from PyKDL import *

import math
import numpy as np
import rospy

r_needle = 0.012 #0.0254/2.0
needle_height_above_tissue = r_needle/math.sqrt(2.0)
d_to_exit_pt = 2*r_needle/math.sqrt(2.0) # only for chosen needle ht
z_tissue = 0.17 # HARD CODED USE ACTUAL TISSUE Z, WRT CAMERA


if __name__ == '__main__':

	#ROS sets-up:
	#node name
    rospy.init_node("needle_planner_test_main")
    print "using tissue z = ", z_tissue

    O_needle = np.zeros(3)
    O_enterance_pt = np.zeros(3)
    exitPoint = np.zeros(3)

    O_needle[2] = z_tissue - needle_height_above_tissue
    print O_needle

    x_enterance = float(raw_input('enter entrance pt x (e.g. -0.02): '))
    y_enterance = float(raw_input('enter entrance pt y (e.g. 0.01): '))

    O_enterance_pt[0] = x_enterance
    O_enterance_pt[1] = y_enterance
    O_enterance_pt[2] = z_tissue

    print "O_enterance_pt = ", O_enterance_pt

    #O_needle = np.array([-0.02,0.004,0.1578]) //we can hard-code O_needle(2) (z-value) for known tissue height
    kvec_yaw = 0.0 #rotation of needle z-axis w/rt camera x-axis

    gripper_affines_wrt_camera = []
    exit_points = []


    v_enterance_to_exit0 = Vector(0, -1, 0) #corresponds to chosen needle kvec along 1,0,0
    v_enterance_to_exit0_numpy = np.array([v_enterance_to_exit0.x(), v_enterance_to_exit0.y(), v_enterance_to_exit0.z()])

    rospy.loginfo(["main: instantiating an object of needle_planner"])

    needlePlanner = needle_planner()

    for kvec_yaw in np.arange(0.0, 6.28, 0.1):
    	v_enterance_to_exit = needlePlanner.Rotz(kvec_yaw)*v_enterance_to_exit0
    	v_enterance_to_exit_numpy = np.array([v_enterance_to_exit.x(), v_enterance_to_exit.y(), v_enterance_to_exit.z()]) #rotate the needle axis about camera z-axis
    	O_exit_pt = O_enterance_pt + d_to_exit_pt * v_enterance_to_exit_numpy 
    	O_needle = 0.5 * (O_exit_pt + O_enterance_pt)

    	O_needle[2] -= needle_height_above_tissue
    	print "O_needle = ", O_needle
    	print "O_exit_pt = ", O_exit_pt
    	print "kvec_yaw = ", kvec_yaw

    	del gripper_affines_wrt_camera[:]
    	
    	successNumber = needlePlanner.simple_horiz_kvec_motion(O_needle, r_needle, kvec_yaw, gripper_affines_wrt_camera)

    	print "total success count: ", successNumber
    	nposes = len(gripper_affines_wrt_camera)

    	rospy.logwarn("at kvec_yaw = %f, computed %d needle-drive gripper poses "%(kvec_yaw,nposes))
    	if (nposes>=40):
    		exitPoint[0] = O_exit_pt[0]
    		exitPoint[1] = O_exit_pt[1]
    		exitPoint[2] = O_exit_pt[2]

    		print "exitPoint: ", exitPoint
    		exit_points.append(exitPoint)
    	#print v_enterance_to_exit
    	