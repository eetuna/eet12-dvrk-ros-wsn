"""
library to compute gripper poses to perform needle driving
# main fnc is: def compute_needle_drive_gripper_affines(...)
# need to specify tissue properties (w/rt camera frame)
# Also specify grasp transform (or accept the default) and choose which arm (or accept default=PSM1)
# provide tissue entrance pt, exit pt and surface normal (w/rt camera frame)
# Get back a vector of affines of corresponding gripper affines, wrt camera frame
# (really, inputs/outputs w/rt whatever frame is of interest, but presumably camera frame)


# NEEDLE-DRIVE ASSUMPTIONS:  
# this code assumes needle driving will be in a circular arc about the needle center
# The needle center is defined as the center of the circle that contains the needle semi-circle
#  This is defined as the origin of the needle frame. The z-axis of the needle frame, bvec_needle,
# is defined perpendicular to the needle plane, with positive direction corresponding to
# needle driving with the tip leading (a requirement) corresponds to positive rotation about
# the needle z axis.
# at present, this code assumes rotation of the needle during driving is about 
# an axis parallel to the tissue surface, i.e. needle z-axis is perpendicular to tissue normal
#  (this is not required, but common)

# NEEDLE GRASP ASSUMPTIONS:
# this code does not care which arm is used; all gripper affines are computed to achieve
# a needle-drive path; not arm IK is performed here

# one can choose where on the circumference of the needle to grab it--but pragmatically,
# one will grab the needle as close to the needle tail as possible (i.e., as far from the 
# needle point as possible), so as to minimize requirements for re-grasping during needle driving
# The choice of where along the circumference to grab the needle is phi_grab_
# Define this as: phi_grab_=0 means to grab the needle in the center of its arc;
# phi_grab_ = +pi/2 means to grab the needle at its tail (should make this pi/2- half_gripper_width
#   to get a full grasp).  phi_grab_ = -pi/2 would mean to grab the gripper tip (not useful for
#  needle driving).

# a common grasp strategy is to hold the needle such that its plane is perpendicular to
# the gripper z-axis, i.e. the needle bvec_needle = +/- bvec_gripper; this is not required in general,
# but it is the default grasp pose for this code

# One must choose the contact points on the gripper where the needle will be held.  The
# default assumption for this code is that the needle will be grabbed half-way between the
# gripper tips and the gripper-jaw joint.  This is changeable via a "set" call.

# With the constraint that the needle plane is perpendicular to the gripper bvec_gripper, there
# are still 4 variations: bvec_needle = +/- bvec_gripper, and needle origin along positive or
# negative gripper y-axis (tvec_gripper).  These may by: 1) selecting direction of bvec_needle
#  (e.g., parallel or antiparallel to bvec_gripper), and 2) choosing "needle_plus_y" or "needle_ninus_y".
# For bvec_needle parallel or antiparallel to bvec_gripper, "needle_plus_y" means that the needle 
# origin will lie on the positive tvec_gripper axis; "needle_minus_y" means that the needle lies on
# the negative tvec_gripper axis.
# More generally, the needle z-axis, bvec_needle, does not have to be parallel or antiparallel to
# the gripper bvec_gripper.  In this case, the needle origin will not lie on the gripper tvec_gripper
# axis.  However, the needle origin would lie in either the positive-y half space of the gripper frame
# or the negative-y half-space of the gripper frame, and the designation "needle_plus_y" would
# still be meaningful.  However, this does not cover the special case where the plane of the needle
# contains bvec_gripper.  In this case, the needle origin will lie at y/gripper = 0.  If this case
# This case is not covered here.


# a common grasp is to have the needle surface-tangent parallel or antiparallel to the gripper x axis at 
#  the grasp location ,i.e. the needle extends out perpendicular to the jaws, i.e.
#  needle tangent  is perpendicular to gripper bvec
# it is also common to have the needle z-axis parallel to or antiparallel to the gripper z axis:
# bvec_needle || bvec_gripper, or bvec_needle || -bvec_gripper
#  That is, the needle origin lies along the (plus or minus) gripper y axis.
# Define the needle-frame x-axis as pointing from needle center (origin) towards needle tip
# Approximately, if grab the needle as far as possible from the needle tip, 
# and this grasp has needle bvec_needle || +/- bvec_gripper, then the needle tip 
# tip would also lie along the
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




class needle_planner:

	def _init_(self):
		"""Constructor.  This initializes a few data members."""
        # data members, event based

        float DEFAULT_NEEDLE_GRASP_DEPTH = 0.005 #default: grab needle at jaw mid-point
		'''this computation is with respect to a gripper--does not matter which arm, since
		# no IK is done here'''
		int GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y=1
		int GRASP_W_NEEDLE_NEGATIVE_GRIPPER_Y=-1
		int GRASP_W_NEEDLE_POSITIVE_GRIPPER_Z=1 #grab needle w/ needle z-axis parallel to gripper z-axis
		int GRASP_W_NEEDLE_NEGATIVE_GRIPPER_Z=-1 # needle z antiparallel to gripper z
		float DEFAULT_NEEDLE_RADIUS = 0.0254/2.0 # for 1" diam needle
		float DEFAULT_NEEDLE_AXIS_HT= DEFAULT_NEEDLE_RADIUS/sqrt(2.0) # height of needle z-axis above tissue
		int NSAMPS_DRIVE_PLAN = 21 # decide how many samples of grasp poses to compute for needle drive over 180 deg
		#phi grab at 0.0--> grab in middle of arc
		float DEFAULT_PHI_GRAB = 0.0# M_PI/2.0; #puts tail of needle in middle of gripper--really not feasible

		#needle properties: these will be constant for a given operation
	    float needle_radius_ 
	    float needle_axis_ht_ # height of needle z-axis above tissue surface
	    float dist_entrance_to_exit_ # follows from needle height and needle radius
	    
	    #needle-grasp properties: these will be constant during needle driving
	    int grab_needle_plus_minus_y_ #needle origin to lie in + or - gripper-y half space?
	    int grab_needle_plus_minus_z_ #bvec_needle = +/- bvec_gripper (for default cases)
	    float grasp_depth_ # w/rt gripper bvec, where are needle-grasp contact points on jaws?
	    float phi_grab_ #helps define needle grasp; phi_grab_=0--> grab at center, phi_grab_=pi/2
	    #--> grab at tail; phi_grab = -pi/2 --> grab at tip
	    #specify needle orientation: e.g., needle z-axis typically parallel or antiparallel to gripper bvec
	    bvec_needle_wrt_grasp_frame_ = Vector()
	    nvec_needle_wrt_grasp_frame_ = Vector()  #nvec_needle is from needle center to needle tip 
	    tvec_needle_wrt_grasp_frame_ = Vector()    
	    #next two transforms are fixed during needle driving; they
	    # describe how the needle is held by the gripper
	    affine_grasp_frame_wrt_gripper_frame_ = Frame()
	    O_needle_frame_wrt_grasp_frame_ = Frame()
	    O_needle_ = Frame() #O_needle is arbitrary frame...e.g. psm1_base
	    R_needle_frame_wrt_grasp_frame_ = Rotation()
	    R0_N_wrt_G_= Rotation()
	    O0_N_wrt_G_ = Vector()
	    affine_needle_frame_wrt_grasp_frame_ = Frame() 
	    affine_needle_frame_wrt_gripper_frame_ = Frame()
	    
	    #these properties require perception: define a frame on the tissue, wrt camera frame
	    nvec_tissue_frame_wrt_camera_ = Vector()
	    tvec_tissue_frame_wrt_camera_ = Vector()
	    bvec_tissue_frame_wrt_camera_ = Vector()
	    desired_needle_entrance_point_ = Vector() # will be specified interactively
	    repaired_exit_pt_ = Vector() #this value must be consistent 
	          #w/ the entrance pt, needle ht and needle radius.  Can't trust operator to get this exact
	    # tissue frame should follow from entrance pt, exit pt and tissue normal
	    R_tissue_frame_wrt_camera_frame_ = Rotation()
	    affine_tissue_frame_wrt_camera_frame_ = Frame() 
	    
	    #needle-driving strategy: follows from 
	    #needle_radius_, needle_axis_ht_, tissue normal, specified entrance pt and repaired exit pt,
	    # assumes needle-z axis is parallel to the tissue, and needle will rotate about its own z axis
	    #bvec_needle must also be perpendicular to tissue-frame x-axis, which points from entrance pt to exit pt
	    # implies bvec_needle MUST point antiparallel to tvec_tissue
	    # this vector will remain constant during needle driving
	    bvec_needle_wrt_tissue_frame_ = Vector() #w/ above assumptions, this is (0,-1,0)
	    #need to define where needle-tip starts;
	    # for simplicity, assume needle tip and needle tail both start at needle_axis_ht_ above the tissue;
	    #  (so needle tip is not yet touching the tissue at the entrance point--overly conservative)
	    # then, needle x-axis (from needle origin to needle tip) is antiparallel to tissue x-axis:
	    nvec_needle_wrt_tissue_frame_ = Vector() #initialize to (-1,0,0); will change orientation during driving
	    tvec_needle_wrt_tissue_frame_ = Vector() # follows from bvec and nvec
	    R0_needle_wrt_tissue_ = Rotation() #initial orientation of needle frame w/rt tissue;
	                                           # follows from above bvec and nvec
	    #specify origin of needle frame (center of needle) w/rt tissue frame; this will be
	    #  directly above the mid-point between entrance and exit pt; will remain constant during
	    # needle driving (in all frames)
	    O_needle_wrt_tissue_ = Vector() #= (dist_entrance_to_exit_, 0, needle_axis_ht_)
	    kvec_needle_ = Vector() # axis of rotation, normal of needle; postive rotation = insertion
	    affine_init_needle_frame_wrt_tissue_ = Frame() #starting pose of needle
	    # variables that evolve during needle driving:    
	    # during driving, insertion angle goes from 0 to pi (could shorten this)
	    float phi_insertion_
	    float psi_needle_axis_tilt_wrt_tissue_
	    R_needle_wrt_tissue_ = Rotation()
	    affine_needle_frame_wrt_tissue_ = Frame()  #this varies during needle driving
	    affine_gripper_frame_wrt_tissue_ = Frame()  # this follows from needle frame and grasp transform
	    
	    affine_needle_frame_wrt_camera_ = Frame()  #this varies during needle driving    
	    #this is the desired result: where should the gripper be to drive the needle
	    # follows from affine_needle_frame_wrt_camera_ and grasp transforms
	    affine_gripper_frame_wrt_camera_frame_ = Frame()    
	    
	    #Davinci_fwd_solver davinci_fwd_solver_; #instantiate a forward-kinematics solver    
	    #Davinci_IK_solver ik_solver_;
	    #default camera transform: should find actual tf by listening, but this
	    # hard-coded default is useful for simple tests
	    default_affine_lcamera_to_psm_one_ = Frame()    
	    default_affine_lcamera_to_psm_two_ = Frame()      


	def set_needle_radius(self, r): #float r
		needle_radius_ = r
	
	def set_needle_axis_ht (self, h): #float h
		needle_axis_ht_ = h
	
	def set_psi_needle_axis_tilt_wrt_tissue(self, tilt):  #float tilt
		psi_needle_axis_tilt_wrt_tissue_ = tilt

	def set_kvec(self, kvec): #PyKDL.Vector kvec
		kvec_needle_ = kvec

	def set_needle_origin(self, O_needle): #PyKDL.Vector O_needle 
		O_needle_ = O_needle
	   
	    # result depends on how the gripper is grasping the needle.  This has a default
	    # grasp transform, changeable with "set" functions
	    #the next 4 fncs change params of the default needle grasp transform

	def set_affine_grasp_frame_wrt_gripper_frame(self, affine): #PyKDL.Frame affine
		affine_grasp_frame_wrt_gripper_frame_ = affine

	def set_affine_needle_frame_wrt_tissue(self, affine): #PyKDL.Frame affine
		affine_needle_frame_wrt_tissue_ = affine

	def set_grasp_depth(self, depth): #float depth
		grasp_depth_ = depth # this far from gripper tip
	    #two kinematic choices, here referred to as "thumb up" or "thumb down"
	    #hmm...maybe redundant with specifying affine_needle_frame_wrt_grasp_frame
	
	def set_grab_needle_plus_minus_y(self, needle_plus_minus_y): #int needle_plus_minus_y
		if (GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y == grab_needle_plus_minus_y_):
			grab_needle_plus_minus_y_= needle_plus_minus_y;
	    elif (GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y == grab_needle_plus_minus_y_):
	    	grab_needle_plus_minus_y_= needle_plus_minus_y;
	    else:
	    	ROS_WARN("grasp status not legal; not being changed")
	
	def set_grab_needle_plus_minus_z(self, needle_plus_minus_z): #int needle_plus_minus_z
	    if (GRASP_W_NEEDLE_POSITIVE_GRIPPER_Z == grab_needle_plus_minus_z_):
	    	grab_needle_plus_minus_z_= needle_plus_minus_z;
	    elif (GRASP_W_NEEDLE_NEGATIVE_GRIPPER_Z == grab_needle_plus_minus_z_): 
	    	grab_needle_plus_minus_z_= needle_plus_minus_z;
	    else: 
	    	ROS_WARN("grasp status needle z-axis sign not legal; not being changed")
	    
	def compute_grasp_transform(self):
	 #if use above sets, apply logic to compute grasp transform
	def compute_grasp_transform(self, phi_x, phi_y): #float phi_x,float phi_y
	
	def set_affine_needle_frame_wrt_gripper_frame(self, affine) #PyKDL.Frame affine
		affine_needle_frame_wrt_gripper_frame_ = affine

	def set_affine_lcamera_to_psm_one(self, affine): #PyKDL.Frame affine
		default_affine_lcamera_to_psm_one_ = affine  

	def set_affine_lcamera_to_psm_two(self, affine): #PyKDL.Frame affine
		default_affine_lcamera_to_psm_two_ = affine   

	def compute_tissue_frame_wrt_camera(self, entrance_pt, exit_pt, tissue_normal): 
		#PyKDL.Vector entrance_pt, PyKDL.Vector exit_pt, PyKDL.Vector tissue_normal
	    #main fnc: given tissue entrance pt, exit pt and surface normal (w/rt camera frame)
	    # compute a sequence of gripper poses (w/rt camera frame) for needle driving

	def compute_needle_drive_gripper_affines(self, gripper_affines_wrt_camera):
		#vector <Eigen::Affine3d> &gripper_affines_wrt_camera

	def simple_compute_needle_drive_gripper_affines(self, gripper_affines):
		#vector <Eigen::Affine3d> &gripper_affines

	def simple_horiz_kvec_motion(self, O_needle, r_needle, kvec_yaw, gripper_affines_wrt_camera):
		#Eigen::Vector3d O_needle, float r_needle, float kvec_yaw, vector <Eigen::Affine3d> &gripper_affines_wrt_camera

	def simple_horiz_kvec_motion_psm2(self, O_needle, r_needle, kvec_yaw, gripper_affines_wrt_camera):
		#Eigen::Vector3d O_needle, float r_needle, float kvec_yaw, vector <Eigen::Affine3d> &gripper_affines_wrt_camera


	    #test fnc--just computes a simple gripper path:
	def simple_test_gripper_motion(self, x, y, z, r, gripper_affines_wrt_camera):
		#float x, float y, float z, float r,vector <Eigen::Affine3d> &gripper_affines_wrt_camera

	def write_needle_drive_affines_to_file(self, gripper_affines_wrt_camera):

	def write_psm2_needle_drive_affines_to_file(self, gripper_affine_psm1, squeeze_cmd, psm2_gripper_affines_wrt_camera):
    #Eigen::Affine3d gripper_affine_psm1, float squeeze_cmd, vector <Eigen::Affine3d> &psm2_gripper_affines_wrt_camera
	    
	def vers(self, phi): #float phi
	    return (1.0-cos(phi)) 
	    
	    #some utility functions:
	    #for rotations about z and y axes
    def Rotz(self, phi): #float phi
    def Roty(self, phi): #float phi
    def Rotx(self, phi): #float phi
    def Rot_k_phi(self, k_vec, phi): #Eigen::Vector3d k_vec,float phi
	
	def print_affine(self, affine): #print out an affine for debug
	#Eigen::Affine3d affine


