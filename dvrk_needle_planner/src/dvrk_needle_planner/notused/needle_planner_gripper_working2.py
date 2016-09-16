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

debug_needle_print = bool('')

#debug_needle_print = True

DEFAULT_NEEDLE_GRASP_DEPTH = 0.005 #default: grab needle at jaw mid-point
'''this computation is with respect to a gripper--does not matter which arm, since
# no IK is done here'''
GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y=1
GRASP_W_NEEDLE_NEGATIVE_GRIPPER_Y=-1
GRASP_W_NEEDLE_POSITIVE_GRIPPER_Z=1 #grab needle w/ needle z-axis parallel to gripper z-axis
GRASP_W_NEEDLE_NEGATIVE_GRIPPER_Z=-1 # needle z antiparallel to gripper z
DEFAULT_NEEDLE_RADIUS = 0.0254/2.0 # for 1" diam needle
DEFAULT_NEEDLE_AXIS_HT= DEFAULT_NEEDLE_RADIUS/math.sqrt(2.0) # height of needle z-axis above tissue
NSAMPS_DRIVE_PLAN = 4 # decide how many samples of grasp poses to compute for needle drive over 180 deg
#phi grab at 0.0--> grab in middle of arc
DEFAULT_PHI_GRAB = 0.0# M_PI/2.0; #puts tail of needle in middle of gripper--really not feasible


class needle_planner:
	def __init__(self):
		"""Constructor.  This initializes a few data members."""
        # data members, event based
		#needle properties: these will be constant for a given operation
		print("needle planner constructor: initializations")

		self.needle_radius_ = DEFAULT_NEEDLE_RADIUS
		self.needle_axis_ht_ = DEFAULT_NEEDLE_AXIS_HT
		self.grab_needle_plus_minus_y_ = GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y
		self.grab_needle_plus_minus_z_ = GRASP_W_NEEDLE_POSITIVE_GRIPPER_Z
		self.grasp_depth_ = DEFAULT_NEEDLE_GRASP_DEPTH
		self.phi_grab_ = DEFAULT_PHI_GRAB

		print "needle arc radius = %f" % self.needle_radius_
		print "needle ht above tissue = %f" % self.needle_axis_ht_

		b_rt = math.sqrt(self.needle_radius_*self.needle_radius_ - self.needle_axis_ht_*self.needle_axis_ht_)
		self.dist_entrance_to_exit_ = 2*b_rt

		print "at needle ht %f, distance from entrance to exit is %f" % (self.needle_axis_ht_, self.dist_entrance_to_exit_)
		self.bvec_needle_wrt_grasp_frame_ = Vector()
		self.nvec_needle_wrt_grasp_frame_ = Vector()
		self.tvec_needle_wrt_grasp_frame_ = Vector()
		R = Rotation(1,0,0, 0,1,0, 0,0,1)

		O_grasp_frame = Vector(0,0,-self.grasp_depth_)
		self.affine_grasp_frame_wrt_gripper_frame_ = Frame(R, O_grasp_frame)
		#print "NeedlePlanner contructor:"
		
		self.O_needle_frame_wrt_grasp_frame_ = Vector()
		O_needle_ = Vector()
		self.R_needle_frame_wrt_grasp_frame_ = Rotation()

		self.affine_needle_frame_wrt_grasp_frame_ = Frame() 
		self.affine_needle_frame_wrt_gripper_frame_ = Frame()

		self.nvec_tissue_frame_wrt_camera_ = Vector()
		self.tvec_tissue_frame_wrt_camera_ = Vector()
		self.bvec_tissue_frame_wrt_camera_ = Vector()

		self.desired_needle_entrance_point_ = Vector()
		self.repaired_exit_pt_ = Vector()

		self.R_tissue_frame_wrt_camera_frame_ = Rotation()
		self.affine_tissue_frame_wrt_camera_frame_ = Frame()


		self.O_needle_wrt_tissue_ = Vector(0.5*self.dist_entrance_to_exit_,0,self.needle_axis_ht_)
		self.bvec_needle_wrt_tissue_frame_ = Vector(0,-1,0)
		self.nvec_needle_wrt_tissue_frame_ = Vector(-1,0,0)
		

		self.tvec_needle_wrt_tissue_frame_ = self.bvec_needle_wrt_tissue_frame_*self.nvec_needle_wrt_tissue_frame_

		self.R0_needle_wrt_tissue_ = Rotation(self.nvec_needle_wrt_tissue_frame_,self.tvec_needle_wrt_tissue_frame_,self.bvec_needle_wrt_tissue_frame_)
		self.affine_init_needle_frame_wrt_tissue_ = Frame(self.R0_needle_wrt_tissue_,self.O_needle_wrt_tissue_)
		print "INIT: initial affine_needle_frame_wrt_tissue_"
		print self.affine_init_needle_frame_wrt_tissue_

		affine_init_needle_frame_wrt_tissue_numpy_ = posemath.toMatrix(self.affine_init_needle_frame_wrt_tissue_)
		

		if(debug_needle_print):
			print "tvec_needle_wrt_tissue_frame_"
			print self.tvec_needle_wrt_tissue_frame_
			print "self.affine_init_needle_frame_wrt_tissue_"
			print self.affine_init_needle_frame_wrt_tissue_;
			print "affine_init_needle_frame_wrt_tissue_numpy_"
			print affine_init_needle_frame_wrt_tissue_numpy_
			print("FIXED: affine_grasp_frame_wrt_gripper_frame_")
			print self.affine_grasp_frame_wrt_gripper_frame_

		self.kvec_needle_ = Vector()
		

		self.R_needle_wrt_tissue_ = Rotation()
		self.affine_needle_frame_wrt_tissue_ = Frame()
		self.affine_gripper_frame_wrt_tissue_ = Frame()
		self.affine_needle_frame_wrt_camera_ = Frame()
		self.affine_gripper_frame_wrt_camera_frame_ = Frame()

		self.compute_grasp_transform()
		self.R0_N_wrt_G_= self.affine_needle_frame_wrt_gripper_frame_.M
		self.O0_N_wrt_G_ = self.affine_needle_frame_wrt_gripper_frame_.p

		default_p_lcamera_to_psm_one_ = Vector(-0.155,-0.03265,0.0)
		default_R_lcamera_to_psm_one_ = Rotation(-1,0,0, 0,1,0, 0,0,-1)
		self.default_affine_lcamera_to_psm_one_ = Frame(default_R_lcamera_to_psm_one_,default_p_lcamera_to_psm_one_)
		self.default_affine_lcamera_to_psm_two_ = Frame()


	def set_needle_radius(self, r): #float r
		self.needle_radius_ = r
	
	def set_needle_axis_ht (self, h): #float h
		self.needle_axis_ht_ = h
	
	def set_psi_needle_axis_tilt_wrt_tissue(self, tilt):  #float tilt
		self.psi_needle_axis_tilt_wrt_tissue_ = tilt

	def set_kvec(self, kvec): #PyKDL.Vector kvec
		self.kvec_needle_ = kvec

	def set_needle_origin(self, O_needle): #PyKDL.Vector O_needle 
		self.O_needle_ = O_needle

	def set_affine_grasp_frame_wrt_gripper_frame(self, affine): #PyKDL.Frame affine
		self.affine_grasp_frame_wrt_gripper_frame_ = affine

	def set_affine_needle_frame_wrt_tissue(self, affine): #PyKDL.Frame affine
		self.affine_needle_frame_wrt_tissue_ = affine

	def set_grasp_depth(self, depth): #float depth
		self.grasp_depth_ = depth # this far from gripper tip

	def set_grab_needle_plus_minus_y(self, needle_plus_minus_y):
		if (GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y == grab_needle_plus_minus_y_):
			self.grab_needle_plus_minus_y_= needle_plus_minus_y
		elif (GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y == grab_needle_plus_minus_y_):
			self.grab_needle_plus_minus_y_= needle_plus_minus_y
		else:
			rospy_logwarn("grasp status not legal; not being changed")

	def print_affine(self, affine):
		print "Rotation: "
		print affine.M
		print "origin: " 
		print affine.p
	
	def set_grab_needle_plus_minus_z(self, needle_plus_minus_z):
		if (GRASP_W_NEEDLE_POSITIVE_GRIPPER_Z == grab_needle_plus_minus_z_):
			self.grab_needle_plus_minus_z_= needle_plus_minus_z
		elif (GRASP_W_NEEDLE_NEGATIVE_GRIPPER_Z == grab_needle_plus_minus_z_):
			self.grab_needle_plus_minus_z_= needle_plus_minus_z
		else:
			rospy_logwarn("grasp status needle z-axis sign not legal; not being changed")

	def compute_grasp_transform(self):
		print("computing grasp transform: ")

		self.O_needle_frame_wrt_grasp_frame_ = Vector(0,self.grab_needle_plus_minus_y_*self.needle_radius_,0)
		self.bvec_needle_wrt_grasp_frame_= Vector(0,0,self.grab_needle_plus_minus_z_)

		self.nvec_needle_wrt_grasp_frame_[0] = self.grab_needle_plus_minus_z_ * self.grab_needle_plus_minus_y_ * math.cos(self.phi_grab_)
		self.nvec_needle_wrt_grasp_frame_[1] = self.grab_needle_plus_minus_y_* math.sin(self.phi_grab_)
		self.nvec_needle_wrt_grasp_frame_[2] = 0.0
		self.tvec_needle_wrt_grasp_frame_ = self.bvec_needle_wrt_grasp_frame_*self.nvec_needle_wrt_grasp_frame_

		self.R_needle_frame_wrt_grasp_frame_ = Rotation(self.nvec_needle_wrt_grasp_frame_,self.tvec_needle_wrt_grasp_frame_,self.bvec_needle_wrt_grasp_frame_)
		#self.R_needle_frame_wrt_grasp_frame_ = self.R_needle_frame_wrt_grasp_frame_.Inverse()
		self.affine_needle_frame_wrt_grasp_frame_ = Frame(self.R_needle_frame_wrt_grasp_frame_,self.O_needle_frame_wrt_grasp_frame_)
		
		if(debug_needle_print):
			print "function compute_grasp_transform"
			print("FIXED: affine_needle_frame_wrt_grasp_frame_")
			print self.affine_needle_frame_wrt_grasp_frame_
			#self.print_affine(self.affine_needle_frame_wrt_grasp_frame_)
		self.affine_needle_frame_wrt_gripper_frame_ = self.affine_grasp_frame_wrt_gripper_frame_*self.affine_needle_frame_wrt_grasp_frame_
		
		if(debug_needle_print):
			print "function compute_grasp_transform"
			print("FIXED: affine_needle_frame_wrt_gripper_frame_")
			print self.affine_needle_frame_wrt_gripper_frame_
	    	#self.print_affine(self.affine_needle_frame_wrt_gripper_frame_)
	   

	def compute_grasp_transform2(self, phi_x, phi_y):

		Rx = self.Rotx(self.phi_x)
		Ry = self.Roty(self.phi_y)
		R_N_wrt_G = Ry*Rx*self.R0_N_wrt_G_
		self.affine_needle_frame_wrt_gripper_frame_ = R_N_wrt_G
		O_N_wrt_G = R_N_wrt_G*self.O0_N_wrt_G_
		self.affine_needle_frame_wrt_gripper_frame_ = O_N_wrt_G
		self.affine_needle_frame_wrt_gripper_frame_ = Frame(R_N_wrt_G,O_N_wrt_G)

		if(debug_needle_print):
			print "function compute_grasp_transform2"
			print("FIXED: affine_needle_frame_wrt_gripper_frame_")
			self.print_affine(self.affine_needle_frame_wrt_gripper_frame_)


	def set_affine_needle_frame_wrt_gripper_frame(self, affine): #PyKDL.Frame affine
		self.affine_needle_frame_wrt_gripper_frame_ = affine

	def set_affine_lcamera_to_psm_one(self, affine):#PyKDL.Frame affine
		self.default_affine_lcamera_to_psm_one_ = affine  

	def set_affine_lcamera_to_psm_two(self, affine):#PyKDL.Frame affine
		self.default_affine_lcamera_to_psm_two_ = affine


	def compute_tissue_frame_wrt_camera(self, entrance_pt, exit_pt, tissue_normal):
		self.bvec_tissue_frame_wrt_camera_ = tissue_normal
		self.nvec_tissue_frame_wrt_camera_ = (exit_pt - entrance_pt)
		nvec_numpy = np.array([self.nvec_tissue_frame_wrt_camera_.x(),self.nvec_tissue_frame_wrt_camera_.y(),self.nvec_tissue_frame_wrt_camera_.z()])
		nvec_numpy_norm = np.linalg.norm(nvec_numpy)

		if(debug_needle_print):
			print "nvec_numpy_norm: "
			print nvec_numpy_norm

		if (nvec_numpy_norm < 0.001):
			rospy_logwarn("specified entrance and exit points are within 1mm; no path will be planned")
			return

		self.nvec_tissue_frame_wrt_camera_ = self.nvec_tissue_frame_wrt_camera_/nvec_numpy_norm;
		self.tvec_tissue_frame_wrt_camera_ = self.bvec_tissue_frame_wrt_camera_*self.nvec_tissue_frame_wrt_camera_
		self.repaired_exit_pt_ = entrance_pt + self.nvec_tissue_frame_wrt_camera_*self.dist_entrance_to_exit_
		self.R_tissue_frame_wrt_camera_frame_ = Rotation(self.nvec_tissue_frame_wrt_camera_,self.tvec_tissue_frame_wrt_camera_,self.bvec_tissue_frame_wrt_camera_)
		#self.R_tissue_frame_wrt_camera_frame_ = self.R_tissue_frame_wrt_camera_frame_.Inverse()
		self.affine_tissue_frame_wrt_camera_frame_ = Frame(self.R_tissue_frame_wrt_camera_frame_,entrance_pt)
		
		if(debug_needle_print):
			print "function compute_tissue_frame_wrt_camera"
			print "FIXED: affine_tissue_frame_wrt_camera_frame_"
			self.print_affine(self.affine_tissue_frame_wrt_camera_frame_)


	def compute_needle_drive_gripper_affines(self, gripper_affines_wrt_camera, gripper_affines_wrt_psm):
	    
	    phi_insertion_ = 0.0
	    self.affine_needle_frame_wrt_tissue_ = self.affine_init_needle_frame_wrt_tissue_

	    if(debug_needle_print):
		    print "function compute_needle_drive_gripper_affines"
		    print "affine_needle_frame_wrt_tissue_.linear()"
		    print self.affine_needle_frame_wrt_tissue_.M
		    print "affine_needle_frame_wrt_tissue_.translation()"
		    print self.affine_needle_frame_wrt_tissue_.p


	    dphi = math.pi/(2.0*(NSAMPS_DRIVE_PLAN-1))
	    

	    R0_needle_wrt_tissue_ = deepcopy(self.affine_needle_frame_wrt_tissue_.M)
	    self.affine_needle_frame_wrt_tissue_.M = self.Rotx(self.psi_needle_axis_tilt_wrt_tissue_)*self.R0_needle_wrt_tissue_
	    
	    if(debug_needle_print):
	    	print "affine_needle_frame_wrt_tissue_.linear()"
	    	print self.affine_needle_frame_wrt_tissue_.M
	    
	    R0_needle_wrt_tissue_ = deepcopy(self.affine_needle_frame_wrt_tissue_.M)

	    kvec_needle_Frame = posemath.toMatrix(self.affine_needle_frame_wrt_tissue_)
	    kvec_needle_numpy = kvec_needle_Frame[0:3,2]
	    kvec_needle = Vector(kvec_needle_numpy[0],kvec_needle_numpy[1],kvec_needle_numpy[2])
	    
	    if(debug_needle_print):
	    	print "function compute_needle_drive_gripper_affines"
	    	print "kvec_needle=" , kvec_needle
	    	print "R0 needle:" , R0_needle_wrt_tissue_


	    needle_origin = self.affine_needle_frame_wrt_tissue_.p
	    if(debug_needle_print):
	    	print self.affine_needle_frame_wrt_tissue_.p
	    self.affine_needle_frame_wrt_tissue_.p = self.Rotx(self.psi_needle_axis_tilt_wrt_tissue_)*needle_origin;

	    if(debug_needle_print):
	    	print "affine_needle_frame_wrt_tissue_.translation()"
	    	print self.affine_needle_frame_wrt_tissue_.p
	    	print "affine_needle_frame_wrt_tissue_.linear()"
	    	print self.affine_needle_frame_wrt_tissue_.M
	    	print "phi_insertion_", phi_insertion_
	    	print "dphi", dphi
	    	print "kvec_needle", kvec_needle

	    for ipose in range(0,NSAMPS_DRIVE_PLAN):
	        Rot_needle = self.Rot_k_phi(kvec_needle,phi_insertion_)
	        if(debug_needle_print):
	        	print "ipose", ipose
	        	print "Rot_needle:", Rot_needle
	        	print "R0_needle_wrt_tissue_",R0_needle_wrt_tissue_
	        #R_needle_wrt_tissue_ = Roty_needle*R0_needle_wrt_tissue_; #update rotation of needle drive
	        self.R_needle_wrt_tissue_ = Rot_needle*R0_needle_wrt_tissue_ #update rotation of needle drive
	        
	        if(debug_needle_print):
	        	print "ipose", ipose
	        	print "function compute_needle_drive_gripper_affines"
	        	print "R_needle w/rt tissue:"
	        	print self.R_needle_wrt_tissue_

	        #need to check these transforms...
	        self.affine_needle_frame_wrt_tissue_.M = self.R_needle_wrt_tissue_
	        self.affine_gripper_frame_wrt_tissue_ = self.affine_needle_frame_wrt_tissue_*self.affine_needle_frame_wrt_gripper_frame_.Inverse()
	        
	        if(debug_needle_print):
	        	print "ipose", ipose
	        	print "function compute_needle_drive_gripper_affines"
	        	print("affine_gripper_frame_wrt_tissue_")
	        	print self.affine_gripper_frame_wrt_tissue_
	        	#self.print_affine(self.affine_gripper_frame_wrt_tissue_)
	        	print("affine_needle_frame_wrt_tissue_")
	        	print self.affine_needle_frame_wrt_tissue_

	        self.affine_gripper_frame_wrt_camera_frame_ = self.affine_tissue_frame_wrt_camera_frame_*self.affine_gripper_frame_wrt_tissue_

	        if(debug_needle_print):
	        	print "ipose", ipose
	        	print "function compute_needle_drive_gripper_affines"
	        	print("affine_gripper_frame_wrt_camera_frame_")
	        	self.print_affine(self.affine_gripper_frame_wrt_camera_frame_)
	        
	        gripper_affines_wrt_camera.append(self.affine_gripper_frame_wrt_camera_frame_)
	        
	        self.affine_gripper_frame_wrt_psm_frame_ = self.default_affine_lcamera_to_psm_one_.Inverse()*self.affine_gripper_frame_wrt_camera_frame_
	        gripper_affines_wrt_psm.append(self.affine_gripper_frame_wrt_psm_frame_)

	        phi_insertion_+=dphi

	        if(debug_needle_print):
		        print "phi_insertion_", phi_insertion_
		        print "dphi", dphi
		        print "kvec_needle", kvec_needle

	    

	def simple_compute_needle_drive_gripper_affines(self, gripper_affines):
	    phi_insertion_ = 0.0

	    nvec=Vector(0,0,1)
	    bvec = self.kvec_needle_
	    tvec = bvec*nvec 

	    #R0_gripper = R0_gripper.Inverse()	
	    dphi = math.pi/((NSAMPS_DRIVE_PLAN-1))
	    Rk = self.Rot_k_phi(self.kvec_needle_,-phi_insertion_)
	    R_gripper = Rk*R0_gripper 
	    O_gripper = self.O_needle_ + self.needle_radius_*Rk*tvec

	    print "Rk: "
	    print Rk
	    print "tvec: " , tvec
	    print "O_needle: " , self.O_needle_
	    print "O_gripper: ", O_gripper
	    
	    self.affine_gripper_wrt_psm1.M = R_gripper
	    self.affine_gripper_wrt_psm1.p = O_gripper
	    
	    for ipose in range(0,NSAMPS_DRIVE_PLAN):

	        Rk = self.Rot_k_phi(self.kvec_needle_,-phi_insertion_)
	        R_gripper = Rk*R0_gripper
	        
	        if(debug_needle_print):
	        	print "R_gripper_wrt_psm1:"
	        if(debug_needle_print):
	        	print R_gripper

	        self.affine_gripper_wrt_psm1.M = R_gripper
	        O_gripper = self.O_needle_ + self.needle_radius_*Rk*tvec
	        self.affine_gripper_wrt_psm1.p = O_gripper
	        
	        print("affine_gripper_wrt_psm1")
	        self.print_affine(self.affine_gripper_wrt_psm1)

	        gripper_affines.append(self.affine_gripper_wrt_psm1)
	        phi_insertion_-=dphi



	def simple_horiz_kvec_motion(self, O_needle, r_needle, kvec_yaw, gripper_affines_wrt_camera, gripper_affines_wrt_psm):
	    dphi = math.pi/40.0
	    bvec0 = Vector(1,0,0)
	    nvec = Vector(0,0,-1)
	    bvec = self.Rotz(kvec_yaw)*bvec0
	    tvec = bvec*nvec
	    R0 = Rotation(nvec,tvec,bvec)
	    #R0 = R0.Inverse()
	    tip_pos = O_needle - r_needle*tvec
	    self.affine_gripper_frame_wrt_camera_frame_.p = tip_pos
	    self.affine_gripper_frame_wrt_camera_frame_.M = R0
	    del gripper_affines_wrt_camera[:]
	    del gripper_affines_wrt_psm[:]
	    nsolns = 0
	    nphi = 0
	    print "nphi: "

	    for phi in range(0,math.pi,dphi):
	        R = self.Rot_k_phi(bvec, phi)*R0
	        self.affine_gripper_frame_wrt_camera_frame_.M=R
	        R_Frame = posemath.toMatrix(self.affine_gripper_frame_wrt_camera_frame_)
	        R_column2_numpy = R_Frame[0:3,1]
	        R_column2 = Vector(R_column2_numpy[0],R_column2_numpy[1],R_column2_numpy[2])
	        tip_pos = O_needle - r_needle*R_column2
	        self.affine_gripper_frame_wrt_camera_frame_.p = tip_pos
	        self.affine_gripper_frame_wrt_psm_frame_ = self.default_affine_lcamera_to_psm_one_.Inverse()*self.affine_gripper_frame_wrt_camera_frame_
	        ''' ERDEM
	        if (ik_solver_.ik_solve(des_gripper1_wrt_base)) 
	        {  nsolns++;
	           cout<<nphi<<",";
	           #cout<<":  found IK; nsolns = "<<nsolns<<endl;
	           gripper_affines_wrt_camera.push_back(affine_gripper_frame_wrt_camera_frame_);
	        }'''
	        gripper_affines_wrt_camera.append(self.affine_gripper_frame_wrt_camera_frame_)
	        gripper_affines_wrt_psm.append(self.affine_gripper_frame_wrt_psm_frame_)
	        nphi += 1

	    print "\n"
	        


	def simple_horiz_kvec_motion_psm2(self, O_needle, r_needle, kvec_yaw, gripper_affines_wrt_camera):
	    dphi = math.pi/40.0
	    bvec0 = Vector(1,0,0)
	    nvec = Vector(0,0,1)
	    bvec = self.Rotz(kvec_yaw)*bvec0
	    tvec = bvec*nvec
	    R0 = Rotation(nvec,tvec,bvec)
	    #R0 = R0.Inverse()
	    tip_pos = O_needle - r_needle*tvec
	    self.affine_gripper_frame_wrt_camera_frame_.p = tip_pos
	    self.affine_gripper_frame_wrt_camera_frame_.M = R0
	    del gripper_affines_wrt_camera[:]
	    del gripper_affines_wrt_psm[:]
	    nsolns = 0
	    nphi = 0
	    print "nphi: "

	    for phi in range(0.0,-math.pi,-dphi):
	        R = self.Rot_k_phi(bvec, phi)*R0
	        self.affine_gripper_frame_wrt_camera_frame_.M=R
	        R_Frame = posemath.toMatrix(self.affine_gripper_frame_wrt_camera_frame_)
	        R_column2_numpy = R_Frame[0:3,1]
	        R_column2 = Vector(R_column2_numpy[0],R_column2_numpy[1],R_column2_numpy[2])
	        tip_pos = O_needle - r_needle*R_column2
	        self.affine_gripper_frame_wrt_camera_frame_.p = tip_pos
	        self.affine_gripper_frame_wrt_psm_frame_ = self.default_affine_lcamera_to_psm_two_.Inverse()*self.affine_gripper_frame_wrt_camera_frame_
	        ''' ERDEM
	        if (ik_solver_.ik_solve(des_gripper1_wrt_base)) 
	        {  nsolns++;
	           cout<<nphi<<",";
	           #cout<<":  found IK; nsolns = "<<nsolns<<endl;
	           gripper_affines_wrt_camera.push_back(affine_gripper_frame_wrt_camera_frame_);
	        }'''
	        gripper_affines_wrt_camera.append(self.affine_gripper_frame_wrt_camera_frame_)
	        gripper_affines_wrt_psm.append(self.affine_gripper_frame_wrt_psm_frame_)
	        nphi += 1

	    print "\n"

	def simple_test_gripper_motion(self, x, y, z, r, gripper_affines_wrt_camera, gripper_affines_wrt_psm):
		print "\n"

	def vers(self, phi):
		return (1.0-math.cos(phi)) 

	def Rotx(self, phi):
	    #Rx_nrow = Vector(1.0, 0.0, 0.0)
	    #Rx_trow = Vector(0.0, math.cos(phi), -math.sin(phi))
	    #Rx_brow = Vector(0.0, math.sin(phi), math.cos(phi))

	    #Rx = Rotation(Rx_nrow,Rx_trow,Rx_brow)
	    Rx = Rotation(1.0,0.0,0.0, 0.0,math.cos(phi),-math.sin(phi), 0.0,math.sin(phi),math.cos(phi))
	    if(debug_needle_print):
	    	print "Rotx:"
	    	print Rx

	    return Rx

	def Roty(self, phi):
		#Ry_nrow = Vector(math.cos(phi), 0.0, math.sin(phi))
		#Ry_trow = Vector(0.0, 1, 0.0)
		#Ry_brow = Vector(-math.sin(phi), 0, math.cos(phi))

		#Ry = Rotation(Ry_nrow,Ry_trow,Ry_brow)
		Ry = Rotation(math.cos(phi),0.0,-math.sin(phi), 0.0,1.0,0.0, -math.sin(phi),0.0,math.cos(phi))

		if(debug_needle_print):
			print "Roty:"
			print Ry

		return Ry

	def Rotz(self, phi):
		#Rz_nrow = Vector(math.cos(phi), -math.sin(phi), 0.0)
		#Rz_trow = Vector(math.sin(phi), math.cos(phi), 0.0)
		#Rz_brow = Vector(0.0, 0.0, 1.0)

		#Rz = Rotation(Rz_nrow,Rz_trow,Rz_brow)
		Rz = Rotation(math.cos(phi),-math.sin(phi),0.0, math.sin(phi),math.cos(phi),0.0, 0.0,0.0,1.0)

		if(debug_needle_print):
			print "Rotz:"
			print Rz

		return Rz

	def Rot_k_phi(self, k_vec, phi): #Eigen::Vector3d k_vec,float phi
	    #Eigen::Matrix3d R_k_phi;
	    kx = k_vec[0]
	    ky = k_vec[1]
	    kz = k_vec[2]
	    
	    #K_nrow = Vector(0.0, -kz, ky)
	    #K_trow = Vector(kz, 0.0, -kx)
	    #K_brow = Vector(-ky, kx, 0.0)

	    N = 3
	    K = Rotation(0.0,-kz,ky, kz,0.0,-kx, -ky,kx,0.0)
	    
	    Ksquare = K*K

	    I = Rotation()
	    
	    K_Frame = Frame(K)
	    
	    K_Frame_numpy = posemath.toMatrix(K_Frame)
	    
	    K_numpy = K_Frame_numpy[0:3,0:3]
	    
	    I_numpy = np.identity(N)
	    K_square_Frame = Frame(Ksquare)
	   

	    K_square_Frame_numpy = posemath.toMatrix(K_square_Frame)
	    
	    K_square_numpy = K_square_Frame_numpy[0:3,0:3]
	    

	    #R_k_phi_numpy = I_numpy + math.sin(phi)*K_numpy + (1-math.cos(phi))*K_numpy*K_numpy
	    #R_k_phi_numpy_FrameTemp = np.c_[R_k_phi_numpy, np.ones(N)]
	    #R_k_phi_numpy_Frame = np.r_[R_k_phi_numpy_FrameTemp,[R_k_phi_numpy_FrameTemp[1]]]
	    #R_k_phi_Frame = posemath.fromMatrix(R_k_phi_numpy_Frame)
	    #R_k_phi = R_k_phi_Frame.M


	    R_k_phi_numpy_square = I_numpy + math.sin(phi)*K_numpy + (1-math.cos(phi))*K_square_numpy
	    R_k_phi_numpy_FrameTemp_square = np.c_[R_k_phi_numpy_square, np.ones(N)]
	    R_k_phi_numpy_Frame_square = np.r_[R_k_phi_numpy_FrameTemp_square,[R_k_phi_numpy_FrameTemp_square[1]]]
	    R_k_phi_Frame_square = posemath.fromMatrix(R_k_phi_numpy_Frame_square)
	    R_k_phi_square = R_k_phi_Frame_square.M
	    

	    if(debug_needle_print):
	    	print "K",K
	    	print "Ksquare",Ksquare
	    	print "K_Frame",K_Frame
	    	print "K_Frame_numpy",K_Frame_numpy
	    	print "K_numpy",K_numpy
	    	print "K_square_Frame",K_square_Frame
	    	print "K_square_Frame_numpy",K_square_Frame_numpy
	    	print "math.sin(phi)",(math.sin(phi))
	    	print "math.sin(phi)",(math.sin(phi))
	    	print "1-math.cos(phi)",(1-math.cos(phi))
	    	print "K_numpy*K_numpy",K_numpy*K_numpy
	    	print "R_k_phi_numpy",R_k_phi_numpy
	    	print "R_k_phi_numpy_FrameTemp",R_k_phi_numpy_FrameTemp
	    	print "R_k_phi_numpy_Frame",R_k_phi_numpy_Frame
	    	print "R_k_phi_Frame",R_k_phi_Frame
	    	print  "R_k_phi",R_k_phi
	    	print "R_k_phi_numpy_square",R_k_phi_numpy_square
	    	print "R_k_phi_numpy_FrameTemp_square",R_k_phi_numpy_FrameTemp_square
	    	print "R_k_phi_numpy_Frame_square",R_k_phi_numpy_Frame_square
	    	print "R_k_phi_Frame_square",R_k_phi_Frame_square
	    	print  "R_k_phi_square",R_k_phi_square

	    return R_k_phi_square

	def write_needle_drive_affines_to_file(self, gripper_affines_wrt_camera, gripper_affines_wrt_psm):

		nposes = len(gripper_affines_wrt_camera)
		if (nposes<1):
			rospy.logwarn("NO POSES TO SAVE")
			return

		str1 = "saving computer %d gripper poses w/rt camera"%nposes
		rospy.loginfo(str1)



		affine_gripper1_frame_wrt_camera_last = gripper_affines_wrt_camera[nposes-1]
		affine_gripper1_frame_wrt_psm_last = self.default_affine_lcamera_to_psm_one_.Inverse()*affine_gripper1_frame_wrt_camera_last

		affine_needle_frame_wrt_psm_last = affine_gripper1_frame_wrt_psm_last*self.affine_needle_frame_wrt_gripper_frame_

		affine_needle_frame_wrt_psm_last_numpy = posemath.toMatrix(affine_needle_frame_wrt_psm_last)
		affine_needle_frame_wrt_psm_last_numpy_R = affine_needle_frame_wrt_psm_last_numpy[0:3,0:3] 
		
		#BELOW do rest of the calculations with numpy array to prevent data type mismatch
		#and convert to PyKDL at the end'''

		nvec_needle = affine_needle_frame_wrt_psm_last_numpy_R[0:,0]
		bvec_needle = affine_needle_frame_wrt_psm_last_numpy_R[0:,2]

		origin_needle = affine_needle_frame_wrt_psm_last.p
		origin_needle_numpy = np.array([origin_needle.x(),origin_needle.y(),origin_needle.z()])
		tip_of_needle_numpy = origin_needle_numpy + self.needle_radius_*nvec_needle
		
		gripper2_grasp_origin_numpy = tip_of_needle_numpy + self.grasp_depth_*bvec_needle
		gripper2_pre_grasp_origin_numpy = gripper2_grasp_origin_numpy - 0.03*bvec_needle
		gripper2_out_of_way_numpy = np.array([0.0, 0.0, -0.02])

		N = 3
		R2_diag_numpy = np.identity(N)
		R2_diag_numpy[2,2] = -1
		R2_diag_numpy[1,1] = -1

		if(debug_needle_print):
			print "R2_diag:"
			print R2_diag_numpy

		R2_diag_numpy_FrameTemp = np.c_[R2_diag_numpy, np.ones(N)]
		R2_diag_numpy_Frame = np.r_[R2_diag_numpy_FrameTemp,[R2_diag_numpy_FrameTemp[1]]]
		R2_diag_Frame = posemath.fromMatrix(R2_diag_numpy_Frame)
		
		#'''BELOW numpy ends here - switch back to PyKDL BELOW'''	

		R2_diag = R2_diag_Frame.M
		gripper2_out_of_way = Vector(gripper2_out_of_way_numpy[0],gripper2_out_of_way_numpy[1],gripper2_out_of_way_numpy[2])
		psm2_nom_wrt_base2 = Frame(R2_diag,gripper2_out_of_way)

		if(debug_needle_print):
			print "affine_lcamera_to_psm_two_: "
			print self.default_affine_lcamera_to_psm_two_.M
			print "origin: "
			print self.default_affine_lcamera_to_psm_two_.p

		psm2_nom_wrt_psm = psm2_nom_wrt_base2

		if(debug_needle_print):
			print "psm2_nom_wrt_psm: "
			print psm2_nom_wrt_psm.M
			print "origin: "
			print psm2_nom_wrt_psm.p

		#'''BELOW switch back to numpy BELOW '''
		
		psm2_nom_wrt_psm_numpy = posemath.toMatrix(psm2_nom_wrt_psm)
		psm2_nom_wrt_psm_numpy_R = psm2_nom_wrt_psm_numpy[0:3,0:3] 

		nvec_gripper2 = psm2_nom_wrt_psm_numpy_R[0:,0]
		tvec_gripper2 = psm2_nom_wrt_psm_numpy_R[0:,1]
		bvec_gripper2 = psm2_nom_wrt_psm_numpy_R[0:,2]

		gripper2_out_of_way_wrt_psm = psm2_nom_wrt_psm.p


		t = 4
		dt = 1

		outfile = open('gripper_poses_in_psm_coords.csp','w')

		for i in range(nposes):
			gripper_affines_wrt_psm_this = gripper_affines_wrt_psm[i]
			#Origin = gripper_affines_wrt_psm_this.p
			#R = gripper_affines_wrt_psm_this.M

			gripper_numpy = posemath.toMatrix(gripper_affines_wrt_psm_this)
			R = gripper_numpy[0:3,0:3] 
			Origin = gripper_numpy[0:3,3]

			outfile.write(str("%0.3f"%Origin[0]) + ', ' + str("%0.3f"%Origin[1]) + ', ' + str("%0.3f"%Origin[2]) + ',     ')

			nvec = R[0:,0]
			tvec = R[0:,1]
			bvec = R[0:,2]

			outfile.write(str("%0.3f"%nvec[0]) + ', ' + str("%0.3f"%nvec[1]) + ', ' + str("%0.3f"%nvec[2]) + ',     ')
			outfile.write(str("%0.3f"%bvec[0]) + ', ' + str("%0.3f"%bvec[1]) + ', ' + str("%0.3f"%bvec[2]) + ',  0,   ')

			outfile.write(str("%0.3f"%gripper2_out_of_way_wrt_psm.x()) + ', ' + str("%0.3f"%gripper2_out_of_way_wrt_psm.y()) + ', ' + str("%0.3f"%gripper2_out_of_way_wrt_psm.z()) + '     ')
			outfile.write(str("%0.3f"%nvec_gripper2[0]) + ', ' + str("%0.3f"%nvec_gripper2[1]) + ', ' + str("%0.3f"%nvec_gripper2[2]) + ',     ')
			outfile.write(str("%0.3f"%bvec_gripper2[0]) + ', ' + str("%0.3f"%bvec_gripper2[1]) + ', ' + str("%0.3f"%bvec_gripper2[2]) + ',  0.0   ' + str("%0.2f"%t) + '\n')


			t+=dt

		outfile.close()
		print("wrote gripper motion plan to file gripper_poses_in_psm_coords.csp")
		rospy.loginfo("wrote gripper motion plan to file gripper_poses_in_psm_coords.csp")


. 	def write_psm2_needle_drive_affines_to_file(self, psm2_gripper_affines_wrt_camera, psm1_gripper_affines_wrt_camera):


		nposes = len(psm2_gripper_affines_wrt_camera)
		if (nposes<1):
			rospy.logwarn("NO POSES TO SAVE")
			return

		str1 = "saving computer %d gripper poses w/rt camera"%nposes
		rospy.loginfo(str1)


		affine_gripper2_frame_wrt_camera_last = psm2_gripper_affines_wrt_camera[nposes-1]
		affine_gripper1_frame_wrt_psm_last_ = self.default_affine_lcamera_to_psm_two_.Inverse()*affine_gripper1_frame_wrt_camera_last
		affine_needle_frame_wrt_psm_last = affine_gripper2_frame_wrt_psm_last*self.affine_needle_frame_wrt_gripper_frame_

		affine_needle_frame_wrt_psm_last_numpy = posemath.toMatrix(affine_needle_frame_wrt_psm_last)
		affine_needle_frame_wrt_psm_last_numpy_R = affine_needle_frame_wrt_psm_last_numpy[0:3,0:3] 
		

		#'''BELOW do rest of the calculations with numpy array to prevent data type mismatch
		#and convert to PyKDL at the end'''

		nvec_needle = affine_needle_frame_wrt_psm_last_numpy_R[0:,0]
		bvec_needle = affine_needle_frame_wrt_psm_last_numpy_R[0:,2]
		tvec_needle = affine_needle_frame_wrt_psm_last_numpy_R[0:,1]

		origin_needle = affine_needle_frame_wrt_psm_last.p
		origin_needle_numpy = np.array([origin_needle.x(),origin_needle.y(),origin_needle.z()])
		tip_of_needle_numpy = origin_needle_numpy + self.needle_radius_*nvec_needle
		
		gripper2_grasp_origin_numpy = tip_of_needle_numpy + self.grasp_depth_*bvec_needle
		gripper2_pre_grasp_origin_numpy = gripper2_grasp_origin_numpy - 0.03*bvec_needle
		gripper2_out_of_way_numpy = np.array([0.14, -0.03, 0.07])

		N = 3
				#R2_diag_numpy = np.identity(N)
		#R2_diag_numpy[2,2] = -1
		#R2_diag_numpy[1,1] = -1

		#print "R2_diag:"
		#print R2_diag_numpy

		#R2_diag_numpy_FrameTemp = np.c_[R2_diag_numpy, np.ones(N)]
		#R2_diag_numpy_Frame = np.r_[R2_diag_numpy_FrameTemp,[R2_diag_numpy_FrameTemp[1]]]
		#R2_diag_Frame = posemath.fromMatrix(R2_diag_numpy_Frame)
		
		#BELOW numpy ends here - switch back to PyKDL BELOW

		#R2_diag = R2_diag_Frame.M
		#gripper2_out_of_way = Vector(gripper2_out_of_way_numpy[0],gripper2_out_of_way_numpy[1],gripper2_out_of_way_numpy[2])
		#psm2_nom_wrt_base2 = Frame(R2_diag,gripper2_out_of_way)


		#print "affine_lcamera_to_psm_two_: "
		#print self.default_affine_lcamera_to_psm_two_.M
		#print "origin: "
		#print self.default_affine_lcamera_to_psm_one_.p

		#psm2_nom_wrt_camera = self.default_affine_lcamera_to_psm_two_*psm2_nom_wrt_base2

		#print "psm2_nom_wrt_camera: "
		#print psm2_nom_wrt_camera.M
		#print "origin: "
		#print psm2_nom_wrt_camera.p

		#BELOW switch back to numpy BELOW
		
		#psm2_nom_wrt_camera_numpy = posemath.toMatrix(psm2_nom_wrt_camera)
		#psm2_nom_wrt_camera_numpy_R = psm2_nom_wrt_camera_numpy[0:3,0:3] 
		
		tvec_gripper2 = nvec_needle
		bvec_gripper2 = bvec_needle
		nvec_gripper2 = np.cross(tvec_gripper2,bvec_gripper2)

		Rtr = np.array([nvec_gripper2, tvec_gripper2, bvec_gripper2])
		print Rtr
		R = np.transpose(Rtr)
		print R

		nvec2 = np.array([1,0,0])
		bvec2 = np.array([0,0,1])

		t = 4
		dt = 1

		outfile = open('psm2_gripper_poses_in_psm_coords.csp','w')
		
		for i in range(nposes):
			psm2_gripper_affines_wrt_psm_this = psm2_gripper_affines_wrt_psm[i]
			#Origin = gripper_affines_wrt_psm_this.p
			#R = gripper_affines_wrt_psm_this.M

			gripper_numpy = posemath.toMatrix(psm2_gripper_affines_wrt_psm_this)
			R = gripper_numpy[0:3,0:3] 
			Origin = gripper_numpy[0:3,3]

			outfile.write(str("%0.3f"%Origin[0]) + ', ' + str("%0.3f"%Origin[1]) + ', ' + str("%0.3f"%Origin[2]) + ',     ')

			nvec = R[0:,0]
			tvec = R[0:,1]
			bvec = R[0:,2]

			outfile.write(str("%0.3f"%nvec[0]) + ', ' + str("%0.3f"%nvec[1]) + ', ' + str("%0.3f"%nvec[2]) + ',     ')
			outfile.write(str("%0.3f"%bvec[0]) + ', ' + str("%0.3f"%bvec[1]) + ', ' + str("%0.3f"%bvec[2]) + ',  0,   ')

			outfile.write(str("%0.3f"%gripper2_out_of_way_numpy[0]) + ', ' + str("%0.3f"%gripper2_out_of_way_numpy[1]) + ', ' + str("%0.3f"%gripper2_out_of_way_numpy[2]) + '     ')
			outfile.write(str("%0.3f"%nvec2[0]) + ', ' + str("%0.3f"%nvec2[1]) + ', ' + str("%0.3f"%nvec2[2]) + ',     ')
			outfile.write(str("%0.3f"%bvec2[0]) + ', ' + str("%0.3f"%bvec2[1]) + ', ' + str("%0.3f"%bvec2[2]) + ',  0.0   ' + str("%0.2f"%t) + '\n')


			t+=dt
		
		
		outfile.close()
		print("wrote gripper motion plan to file psm2_gripper_poses_in_psm_coords.csp") 
		rospy.loginfo("wrote gripper motion plan to file psm2_gripper_poses_in_psm_coords.csp") 