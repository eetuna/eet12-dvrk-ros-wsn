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

from numpy import np
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

debug_needle_print = bool('')
DEFAULT_NEEDLE_GRASP_DEPTH = 0.005 #default: grab needle at jaw mid-point
'''this computation is with respect to a gripper--does not matter which arm, since
# no IK is done here'''
GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y=1
GRASP_W_NEEDLE_NEGATIVE_GRIPPER_Y=-1
GRASP_W_NEEDLE_POSITIVE_GRIPPER_Z=1 #grab needle w/ needle z-axis parallel to gripper z-axis
GRASP_W_NEEDLE_NEGATIVE_GRIPPER_Z=-1 # needle z antiparallel to gripper z
DEFAULT_NEEDLE_RADIUS = 0.0254/2.0 # for 1" diam needle
DEFAULT_NEEDLE_AXIS_HT= DEFAULT_NEEDLE_RADIUS/sqrt(2.0) # height of needle z-axis above tissue
NSAMPS_DRIVE_PLAN = 21 # decide how many samples of grasp poses to compute for needle drive over 180 deg
#phi grab at 0.0--> grab in middle of arc
DEFAULT_PHI_GRAB = 0.0# M_PI/2.0; #puts tail of needle in middle of gripper--really not feasible


class needle_planner:
	def _init_(self):
		"""Constructor.  This initializes a few data members."""
        # data members, event based
        rospy.loginfo("needle planner constructor: initializations")

		#needle properties: these will be constant for a given operation
		self.needle_radius_ = DEFAULT_NEEDLE_RADIUS
	    self.needle_axis_ht_ = DEFAULT_NEEDLE_AXIS_HT # height of needle z-axis above tissue surface
	    
	    
	    #needle-grasp properties: these will be constant during needle driving
	    self.grab_needle_plus_minus_y_ = GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y #needle origin to lie in + or - gripper-y half space?
	    self.grab_needle_plus_minus_z_ = GRASP_W_NEEDLE_POSITIVE_GRIPPER_Z #bvec_needle = +/- bvec_gripper (for default cases)
	    self.grasp_depth_ = DEFAULT_NEEDLE_GRASP_DEPTH # w/rt gripper bvec, where are needle-grasp contact points on jaws?
	    self.phi_grab_ = DEFAULT_PHI_GRAB#helps define needle grasp; phi_grab_=0--> grab at center, phi_grab_=pi/2
	    #--> grab at tail; phi_grab = -pi/2 --> grab at tip
   		rospy.loginfo("needle arc radius = %f",needle_radius_)
		rospy.loginfo("needle ht above tissue = %f",needle_axis_ht_)
		#compute entrance-to-exit distance, based on needle radius and needle-axis height
		#consider equilateral triangle, needle origin above tissue (base) at ht h
		#sides of triangle are both r
		#two right triangles w/ height h and hypoteneus r--> b_rt (right-triangle base) = sqrt(r^2 - h^2)
		#base of equilateral triangle = 2*b_rt    
		b_rt = sqrt(self.needle_radius_*self.needle_radius_ - self.needle_axis_ht_*self.needle_axis_ht_);
    	self.dist_entrance_to_exit_ = 2*b_rt # follows from needle height and needle radius
    	rospy.loginfo("at needle ht %f, distance from entrance to exit is %f",needle_axis_ht_,dist_entrance_to_exit_)	


	    #specify needle orientation: e.g., needle z-axis typically parallel or antiparallel to gripper bvec
	    self.bvec_needle_wrt_grasp_frame_ = Vector()
	    self.nvec_needle_wrt_grasp_frame_ = Vector()  #nvec_needle is from needle center to needle tip 
	    self.tvec_needle_wrt_grasp_frame_ = Vector()    
	    #next two transforms are fixed during needle driving; they
	    # describe how the needle is held by the gripper
	    R = Rotation(1,0,0, 0,1,0, 0,0,1)

	    O_grasp_frame = Vector(0,0,-self.grasp_depth_)
	    self.affine_grasp_frame_wrt_gripper_frame_ = Frame(R, O_grasp_frame)
	    rospy.loginfo("FIXED: affine_grasp_frame_wrt_gripper_frame_")
        #print_affine(affine_grasp_frame_wrt_gripper_frame_); 

	    self.O_needle_frame_wrt_grasp_frame_ = Frame()
	    O_needle_ = Frame() #O_needle is arbitrary frame...e.g. psm1_base
	    self.R_needle_frame_wrt_grasp_frame_ = Rotation()
	    self.R0_N_wrt_G_= self.affine_needle_frame_wrt_gripper_frame_.M
	    self.O0_N_wrt_G_ = self.affine_needle_frame_wrt_gripper_frame_.p
	    self.affine_needle_frame_wrt_grasp_frame_ = Frame() 
	    self.affine_needle_frame_wrt_gripper_frame_ = Frame()
	    
	    #these properties require perception: define a frame on the tissue, wrt camera frame
	    self.nvec_tissue_frame_wrt_camera_ = Vector()
	    self.tvec_tissue_frame_wrt_camera_ = Vector()
	    self.bvec_tissue_frame_wrt_camera_ = Vector()
	    self.desired_needle_entrance_point_ = Vector() # will be specified interactively
	    self.repaired_exit_pt_ = Vector() #this value must be consistent 
	          #w/ the entrance pt, needle ht and needle radius.  Can't trust operator to get this exact
	    # tissue frame should follow from entrance pt, exit pt and tissue normal
	    self.R_tissue_frame_wrt_camera_frame_ = Rotation()
	    self.affine_tissue_frame_wrt_camera_frame_ = Frame() 
	    
	    #needle-driving strategy: follows from 
	    #needle_radius_, needle_axis_ht_, tissue normal, specified entrance pt and repaired exit pt,
	    # assumes needle-z axis is parallel to the tissue, and needle will rotate about its own z axis
	    #bvec_needle must also be perpendicular to tissue-frame x-axis, which points from entrance pt to exit pt
	    # implies bvec_needle MUST point antiparallel to tvec_tissue
	    # this vector will remain constant during needle driving
	    self.bvec_needle_wrt_tissue_frame_ = Vector(0,-1,0) #w/ above assumptions, this is (0,-1,0)
	    #need to define where needle-tip starts;
	    # for simplicity, assume needle tip and needle tail both start at needle_axis_ht_ above the tissue;
	    #  (so needle tip is not yet touching the tissue at the entrance point--overly conservative)
	    # then, needle x-axis (from needle origin to needle tip) is antiparallel to tissue x-axis:
	    self.nvec_needle_wrt_tissue_frame_ = Vector(-1,0,0) #initialize to (-1,0,0); will change orientation during driving
	    self.tvec_needle_wrt_tissue_frame_ = self.bvec_needle_wrt_tissue_frame_*self.nvec_needle_wrt_tissue_frame_ # follows from bvec and nvec
	    self.R0_needle_wrt_tissue_ = Rotation(self.nvec_needle_wrt_tissue_frame_,self.tvec_needle_wrt_tissue_frame_,self.bvec_needle_wrt_tissue_frame_) #initial orientation of needle frame w/rt tissue;
	                                           # follows from above bvec and nvec


		# R.Inverse() == transpose R
		self.R0_needle_wrt_tissue_ = self.R0_needle_wrt_tissue_.Inverse()

	    #specify origin of needle frame (center of needle) w/rt tissue frame; this will be
	    #  directly above the mid-point between entrance and exit pt; will remain constant during
	    # needle driving (in all frames)
	    self.O_needle_wrt_tissue_ = Vector(0.5*self,dist_entrance_to_exit_,0,self,needle_axis_ht_) #= (dist_entrance_to_exit_, 0, needle_axis_ht_)
	    self.kvec_needle_ = Vector() # axis of rotation, normal of needle; postive rotation = insertion
	    self.affine_init_needle_frame_wrt_tissue_ = Frame(self.R0_needle_wrt_tissue_,self.O_needle_wrt_tissue_) #starting pose of needle
	    # variables that evolve during needle driving:    
	    # during driving, insertion angle goes from 0 to pi (could shorten this)
	    #float phi_insertion_
	    #float psi_needle_axis_tilt_wrt_tissue_
	    self.R_needle_wrt_tissue_ = Rotation()
	    self.affine_needle_frame_wrt_tissue_ = Frame()  #this varies during needle driving
	    self.affine_gripper_frame_wrt_tissue_ = Frame()  # this follows from needle frame and grasp transform
	    
	    self.affine_needle_frame_wrt_camera_ = Frame()  #this varies during needle driving    
	    #this is the desired result: where should the gripper be to drive the needle
	    # follows from affine_needle_frame_wrt_camera_ and grasp transforms
	    self.affine_gripper_frame_wrt_camera_frame_ = Frame()    
	    
	    #Davinci_fwd_solver davinci_fwd_solver_; #instantiate a forward-kinematics solver    
	    #Davinci_IK_solver ik_solver_;
	    #default camera transform: should find actual tf by listening, but this
	    # hard-coded default is useful for simple tests
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
		kvec_needle_ = kvec

	def set_needle_origin(self, O_needle): #PyKDL.Vector O_needle 
		self.O_needle_ = O_needle
	   
	    # result depends on how the gripper is grasping the needle.  This has a default
	    # grasp transform, changeable with "set" functions
	    #the next 4 fncs change params of the default needle grasp transform

	def set_affine_grasp_frame_wrt_gripper_frame(self, affine): #PyKDL.Frame affine
		self.affine_grasp_frame_wrt_gripper_frame_ = affine

	def set_affine_needle_frame_wrt_tissue(self, affine): #PyKDL.Frame affine
		self.affine_needle_frame_wrt_tissue_ = affine

	def set_grasp_depth(self, depth): #float depth
		self.grasp_depth_ = depth # this far from gripper tip
	    #two kinematic choices, here referred to as "thumb up" or "thumb down"
	    #hmm...maybe redundant with specifying affine_needle_frame_wrt_grasp_frame
	
	def set_grab_needle_plus_minus_y(self, needle_plus_minus_y): #int needle_plus_minus_y
		if (GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y == grab_needle_plus_minus_y_):
			self.grab_needle_plus_minus_y_= needle_plus_minus_y;
	    elif (GRASP_W_NEEDLE_POSITIVE_GRIPPER_Y == grab_needle_plus_minus_y_):
	    	self.grab_needle_plus_minus_y_= needle_plus_minus_y;
	    else:
	    	ROS_WARN("grasp status not legal; not being changed")
	
	def set_grab_needle_plus_minus_z(self, needle_plus_minus_z): #int needle_plus_minus_z
	    if (GRASP_W_NEEDLE_POSITIVE_GRIPPER_Z == grab_needle_plus_minus_z_):
	    	self.grab_needle_plus_minus_z_= needle_plus_minus_z;
	    elif (GRASP_W_NEEDLE_NEGATIVE_GRIPPER_Z == grab_needle_plus_minus_z_): 
	    	self.grab_needle_plus_minus_z_= needle_plus_minus_z;
	    else: 
	    	ROS_WARN("grasp status needle z-axis sign not legal; not being changed")
	    
	def compute_grasp_transform(self):
		rospy.loginfo("computing grasp transform: ")
		self.O_needle_frame_wrt_grasp_frame_ = Vector(0,self.grab_needle_plus_minus_y_*self.needle_radius_,0)
		self.bvec_needle_wrt_grasp_frame_= Vector(0,0,self.grab_needle_plus_minus_z_)
		'''--> rot gripper about +z gripper axis to insert needle
	    # compute the needle x-axis relative to the gripper;
	    # needle x-axis is from needle origin to needle tip
	    # handle the 4 default case here:
	    # gripper and needle z axes are parallel or antiparallel (sign of grab_needle_plus_minus_z_)
	    # needle origin is in + or - y half space of gripper frame (sign of grab_needle_plus_minus_y_)
	    # grab the needle at phi_grab_, defined as 0 at center of needle, +pi/2 at tail of needle
	    '''
	    self.nvec_needle_wrt_grasp_frame_(0) = self.grab_needle_plus_minus_z_*self.grab_needle_plus_minus_y_*cos(self.phi_grab_)
	    self.nvec_needle_wrt_grasp_frame_(1) = self.grab_needle_plus_minus_y_*sin(self.phi_grab_)
	    self.nvec_needle_wrt_grasp_frame_(2) = 0.0;
	    self.tvec_needle_wrt_grasp_frame_ = self.bvec_needle_wrt_grasp_frame_*self.nvec_needle_wrt_grasp_frame_
	    
	    self.R_needle_frame_wrt_grasp_frame_ = Rotation(self.nvec_needle_wrt_grasp_frame_,self.tvec_needle_wrt_grasp_frame_,self.bvec_needle_wrt_grasp_frame_)
 		self.R_needle_frame_wrt_grasp_frame_  = self.R_needle_frame_wrt_grasp_frame_.Inverse
 		self.affine_needle_frame_wrt_grasp_frame = Frame(R_needle_frame_wrt_grasp_frame_,O_needle_frame_wrt_grasp_frame_)
	    if(debug_needle_print):
	    	rospy.loginfo("FIXED: affine_needle_frame_wrt_grasp_frame_")
	    if(debug_needle_print):
	    	print_affine(self.affine_needle_frame_wrt_grasp_frame_) 
	    	self.affine_needle_frame_wrt_gripper_frame_ = self.affine_grasp_frame_wrt_gripper_frame_*self.affine_needle_frame_wrt_grasp_frame_
	    if(debug_needle_print):
	    	rospy.loginfo("FIXED: affine_needle_frame_wrt_gripper_frame_")
	    if(debug_needle_print):
	    	print_affine(self.affine_needle_frame_wrt_gripper_frame_) 
    


	 #if use above sets, apply logic to compute grasp transform
	def compute_grasp_transform(self, phi_x, phi_y): #float phi_x,float phi_y
		#Eigen::Matrix3d R_N_wrt_G,Rx,Ry;
	    #Eigen::Vector3d O_N_wrt_G;
	    Rx =  self.Rotx(self.phi_x);
	    Ry = self.Roty(self.phi_y);
	    R_N_wrt_G = Ry*Rx*self.R0_N_wrt_G_;
	    self.affine_needle_frame_wrt_gripper_frame_ = R_N_wrt_G
	    O_N_wrt_G = R_N_wrt_G*self.O0_N_wrt_G_;
	    self.affine_needle_frame_wrt_gripper_frame_ = O_N_wrt_G
	    self.affine_needle_frame_wrt_gripper_frame_ = Frame(R_N_wrt_G,O_N_wrt_G)
	    if(debug_needle_print):
	    	rospy.loginfo("FIXED: affine_needle_frame_wrt_gripper_frame_")
	    if(debug_needle_print):
	    	print_affine(self.affine_needle_frame_wrt_gripper_frame_)
	
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
	    #set up tissue frame w/rt camera
	    self.bvec_tissue_frame_wrt_camera_ = tissue_normal;
	    self.nvec_tissue_frame_wrt_camera_ = (exit_pt - entrance_pt);
	    #normalize this vector:
	    nvec_norm = np.linalg.norm(self.nvec_tissue_frame_wrt_camera_);
	    if (nvec_norm < 0.001):
	    	ROS_WARN("specified entrance and exit points are within 1mm; no path will be planned")
	    	return
	
	    self.nvec_tissue_frame_wrt_camera_ = nvec_tissue_frame_wrt_camera_/nvec_norm;
	    self.tvec_tissue_frame_wrt_camera_ = bvec_tissue_frame_wrt_camera_*self.nvec_tissue_frame_wrt_camera_
	    self.repaired_exit_pt_ = entrance_pt + self.nvec_tissue_frame_wrt_camera_*self.dist_entrance_to_exit_
	    self.R_tissue_frame_wrt_camera_frame_ = Rotation(self.nvec_tissue_frame_wrt_camera_,self.tvec_tissue_frame_wrt_camera_,self.bvec_tissue_frame_wrt_camera_)
	    self.R_tissue_frame_wrt_camera_frame_ = self.R_tissue_frame_wrt_camera_frame_.Inverse
	    self.affine_tissue_frame_wrt_camera_frame_ = Frame(self.R_tissue_frame_wrt_camera_frame_,entrance_pt)

	    if(debug_needle_print):
	    	print "FIXED: affine_tissue_frame_wrt_camera_frame_"
	    if(debug_needle_print):
	    	print_affine(affine_tissue_frame_wrt_camera_frame_)


	#main fnc: 
	#Return a vector full of affines describing desired gripper frames w/rt camera frame
	def compute_needle_drive_gripper_affines(self, gripper_affines_wrt_camera):
		#vector <Eigen::Affine3d> &gripper_affines_wrt_camera
		'''
		#must first compute the tissue frame and its transform w/rt camera frame
	    # by calling: compute_tissue_frame_wrt_camera
	    
	    #next, establish the initial needle frame w/rt tissue frame:
	    #per constructor, have a default initial needle frame w/rt tissue frame
	    # override this, if desired, via fnc 
	    #rotate needle frame about needle z-axis;   
	    #sample this path in angular increments dphi
	    #given R_needle_frame_wrt_tissue_frame, the needle z-axis IS anti-parallel to the
	    #tissue-frame y-axis: bvec_needle_wrt_tissue_frame_<<0,-1,0;
	    # take the needle-frame R_needle_wrt_tissue, and rotate it about the tissue-frame y-axis
	    # keep the needle origin constant
	    
	    #next two are variable, as the needle is inserted:
	    #initialize consistent insertion angle and initial pose of needle w/rt tissue;
	    # these values will change during needle driving:'''
	    phi_insertion_ = 0.0; #start drive from here   
    	self.affine_needle_frame_wrt_tissue_ = self.affine_init_needle_frame_wrt_tissue_

	    #version for 90-deg drive only ********
	    dphi = math.pi/(2.0*(NSAMPS_DRIVE_PLAN-1));  #M_PI/(NSAMPS_DRIVE_PLAN-1);
	    #Eigen::Vector3d kvec_needle;
	    #kvec_needle = affine_needle_frame_wrt_tissue_.linear().col(2); #z-axis of needle frame
	    
	    #kvec_needle = Vector()
	    #Rot_needle=Rotation()
	    self.R0_needle_wrt_tissue_ = self.affine_needle_frame_wrt_tissue_.p #update this, in case user changed init needle pose
	    #rotate the needle about the tissue-frame x-axis to tilt the needle bvec:
	    self.affine_needle_frame_wrt_tissue_.M = self.Rotx(self.psi_needle_axis_tilt_wrt_tissue_)*self.R0_needle_wrt_tissue_
	    self.R0_needle_wrt_tissue_= self.affine_needle_frame_wrt_tissue_.M
	    kvec_needle_Frame = posemath.toMatrix(self.affine_needle_frame_wrt_tissue_)
	    kvec_needle_numpy = kvec_needle_Frame[0:3,2]
	    kvec_needle = Vector(kvec_needle_numpy[0],kvec_needle_numpy[1],kvec_needle_numpy[2])
	    if(debug_needle_print):
	    	print "kvec_needle=" , kvec_needle
	    if(debug_needle_print):
	    	print "R0 needle:"
	    
	    needle_origin = self.affine_needle_frame_wrt_tissue_.p;
	    self.affine_needle_frame_wrt_tissue_.p = self.Rotx(self.psi_needle_axis_tilt_wrt_tissue_)*needle_origin;
	    if(debug_needle_print):
	    	print self.affine_needle_frame_wrt_tissue_.M

	    for ipose in range(0,NSAMPS_DRIVE_PLAN):
	        #Roty_needle = Roty(-phi_insertion_); #rotate about tissue-frame -y axis
	        #more general--allow any needle z axis:
	        Rot_needle = self.Rot_k_phi(kvec_needle,phi_insertion_)
	        #R_needle_wrt_tissue_ = Roty_needle*R0_needle_wrt_tissue_; #update rotation of needle drive
	        self.R_needle_wrt_tissue_ = Rot_needle*self.R0_needle_wrt_tissue_ #update rotation of needle drive
	        
	        if(debug_needle_print):
	        	print "R_needle w/rt tissue:"
	        if(debug_needle_print):
	        	print self.R_needle_wrt_tissue_
	        #need to check these transforms...
	        self.affine_needle_frame_wrt_tissue_.M = self.R_needle_wrt_tissue_
	        #rospy.loginfo("affine_needle_frame_wrt_tissue_");
	        #print_affine(affine_needle_frame_wrt_tissue_);
	        
	        self,affine_gripper_frame_wrt_tissue_ = 
	                 self.affine_needle_frame_wrt_tissue_*self.affine_needle_frame_wrt_gripper_frame_.Inverse()
	        
	        if(debug_needle_print):
	        	rospy.loginfo("affine_gripper_frame_wrt_tissue_")
	        if(debug_needle_print):
	        	print_affine(self.affine_gripper_frame_wrt_tissue_)
	        
	        #affine_needle_frame_wrt_camera_ = affine_tissue_frame_wrt_camera_frame_.inverse()*affine_needle_frame_wrt_tissue_;  
	        #rospy.loginfo("affine_needle_frame_wrt_camera_");
	        #print_affine(affine_needle_frame_wrt_camera_); 
	        
	        self.affine_gripper_frame_wrt_camera_frame_ = 
	                self.affine_tissue_frame_wrt_camera_frame_*self.affine_gripper_frame_wrt_tissue_


	        if(debug_needle_print):
	        	rospy.loginfo("affine_gripper_frame_wrt_camera_frame_")
	        if(debug_needle_print):
	        	print_affine(self.affine_gripper_frame_wrt_camera_frame_)
	        
	        gripper_affines_wrt_camera.append(self.affine_gripper_frame_wrt_camera_frame_);
	        
	                phi_insertion_+=dphi; 
	        
	'''
    #simple version: don't worry about tissue frame--and assumes affines will be expressed w/rt psm1_base frame;
	#also assume reverse motion--"un-driving" needle, with final state at gripper just touching tissue
	# don't worry about where to grab needle
	# assume needle rotations are nominal case--angles=0, i.e. gripper
	# should end up with gripper nvec parallel to psm1 bvec (??)
	# need to specify yaw of kvec and origin of needle center
	# use "set" fncs to set kvec_needle_ and O_needle_ FIRST'''
	def simple_compute_needle_drive_gripper_affines(self, gripper_affines):
		#vector <Eigen::Affine3d> &gripper_affines

	    phi_insertion_ = 0.0; #start drive from here   
	    #Eigen::Affine3d affine_gripper_wrt_psm1;    
	    #Eigen::Matrix3d R_gripper,R0_gripper,Rot_needle,Rk;
	    #Eigen::Vector3d nvec,tvec,bvec,O_gripper;
	    nvec=Vector(0,0,1) # gripper x-axis pointing parallel to z-axis of psm1 
	    bvec = self.kvec_needle_  # gripper z-axis parallel to needle axis of rotation--special case
	    tvec = bvec*nvec  #tvec of gripper at init pose points from needle origin to gripper origin
	    R0_gripper =  Rotation(nvec,bvec,tvec)
	    R0_gripper = R0_gripper.Inverse()	
	    
	    dphi = math.pi/((NSAMPS_DRIVE_PLAN-1))  #M_PI/(NSAMPS_DRIVE_PLAN-1);
	    Rk = self.Rot_k_phi(self.kvec_needle_,-self.phi_insertion_)
	    R_gripper = Rk*R0_gripper 
	    O_gripper = self.O_needle_ + self.needle_radius_*Rk*tvec
	    print "Rk: "
	    print Rk
	    print "tvec: " , tvec
	    print "O_needle: " , self.O_needle_
	    print "O_gripper: ", O_gripper
	    
	    #rotate the needle about the needle kvec...in negative direction to "un" drive needle:
	    self.affine_gripper_wrt_psm1.M = R_gripper
	    self.affine_gripper_wrt_psm1.p = O_gripper
	    
	    #if(debug_needle_print) cout<<"kvec_needle="<<kvec_needle.transpose()<<endl;
	    #if(debug_needle_print) cout<<"R0 needle:"<<endl;
	    
	    #if(debug_needle_print) cout<<affine_gripper_wrt_psm1.linear()<<endl;

	    for ipose in range(0,NSAMPS_DRIVE_PLAN):
	        #Roty_needle = Roty(-phi_insertion_); #rotate about tissue-frame -y axis
	        #more general--allow any needle z axis:
	        Rk = self.Rot_k_phi(self.kvec_needle_,-self.phi_insertion_)
	        #R_needle_wrt_tissue_ = Roty_needle*R0_needle_wrt_tissue_; #update rotation of needle drive
	        R_gripper = Rk*R0_gripper #update rotation of needle drive
	        
	        if(debug_needle_print):
	        	print "R_gripper_wrt_psm1:"
	        if(debug_needle_print):
	        	print R_gripper

	        self.affine_gripper_wrt_psm1.M = R_gripper
	        O_gripper = self.O_needle_ + self.needle_radius_*Rk*tvec
	        self.affine_gripper_wrt_psm1.p = O_gripper
	        
	        rospy.loginfo("affine_gripper_wrt_psm1")
	        self.print_affine(self.affine_gripper_wrt_psm1)
	      
	        gripper_affines.append(self.affine_gripper_wrt_psm1)
	        
	        phi_insertion_-=dphi
	    

	'''
	# another simple diagnostic function:
	# generate a sequence of gripper1 poses that have the gripper (and needle axis)
	# point horizontal (parallel to tissue) at angle kvec_yaw (from the x-axis)
	# needle center is at O_needle, and needle radius is r_needle
	# all poses are w/rt camera frame'''
	def simple_horiz_kvec_motion(self, O_needle, r_needle, kvec_yaw, gripper_affines_wrt_camera):
		#Eigen::Vector3d O_needle, float r_needle, float kvec_yaw, vector <Eigen::Affine3d> &gripper_affines_wrt_camera
		#double phi_circle = 0.0;
	    dphi = math.pi/40.0
	    #Eigen::Matrix3d R,R0;
	    #Eigen::Vector3d nvec,tvec,bvec,bvec0,tip_pos;
	    #Eigen::Affine3d des_gripper1_wrt_base;
	    bvec0=Vector(1,0,0)#at kvec_yaw=0, set needle axis parallel to camera-frame x-axis
	    #nvec<<-1,0,0;
	    #bvec<<1,0,0;
	    nvec=Vector(0,0,-1)  #start w/ gripper x axis pointing "up" towards camera 
	    bvec = self.Rotz(kvec_yaw)*bvec0 #rotate the needle axis about camera z-axis
	    #cout<<"needle z-vec in camera frame: "<<bvec.transpose()<<endl;
	            
	    tvec = bvec*nvec
	    R0 = Rotation(nvec,tvec,bvec)
	    R0 = R0.Inverse()
	    tip_pos=O_needle - r_needle*tvec
	    self.affine_gripper_frame_wrt_camera_frame_.p = tip_pos
	    self.affine_gripper_frame_wrt_camera_frame_.M = R0
	    #cout<<"start gripper1 orientation: "<<endl;
	    #cout<<affine_gripper_frame_wrt_camera_frame_.linear()<<endl;
	    #cout<<"needle tip: "<<affine_gripper_frame_wrt_camera_frame_.translation().transpose()<<endl;
	    del gripper_affines_wrt_camera[:]
	    nsolns = 0
	    nphi = 0
	    print "nphi: "
	    for phi in range(0,math.pi,dphi):
	        #need to rotate gripper frame about camera-frame x-axis;
	        # DO want to describe this as equiv rotation about gripper-frame z-axis
	        
	        #careful here: R0 is R_gripper/camera
	        # rotate about the camera-frame x-axis;
	        # more generally, rotate about k_vec, expressed in camera coords
	        #R = Rotx(phi)*R0; #Rot_k_phi(bvec, phi)*R0;
	        R = self.Rot_k_phi(bvec, phi)*R0
	        #cout<<"Rotx(phi):"<<endl;
	        #cout<<Rotx(phi)<<endl;
	        #cout<<"Rot_k_phi(bvec,phi):"<<endl;
	        #cout<<Rot_k_phi(bvec,phi)<<endl;
	        self.affine_gripper_frame_wrt_camera_frame_.M=R
	        R_Frame = posemath.toMatrix(self.affine_gripper_frame_wrt_camera_frame_)
	  		R_column2_numpy = R_Frame[0:3,1]
	  		R_column2 = Vector(R_column2_numpy[0],R_column2_numpy[1],R_column2_numpy[2])
	        tip_pos = O_needle - r_needle*R_column2 # assumes needle x-axis is parallel to gripper y axis
	        self.affine_gripper_frame_wrt_camera_frame_.p = tip_pos
	        #cout<<"gripper1 orientation at phi = "<<phi<<endl;
	        #cout<<affine_gripper_frame_wrt_camera_frame_.linear()<<endl;
	        #cout<<"needle tip: "<<affine_gripper_frame_wrt_camera_frame_.translation().transpose()<<endl;      
	        #gripper_affines_wrt_camera.push_back(affine_gripper_frame_wrt_camera_frame_);
	        #cout<<"enter 1: ";
	        #int ans;
	        #cin>>ans;
	        #express in psm base frame
	        des_gripper1_wrt_base = self.default_affine_lcamera_to_psm_one_.Inverse()*self.affine_gripper_frame_wrt_camera_frame_;
	        #try computing IK:
	        #cout<<"phi = "<<phi;
	        ''' ERDEM
	        if (ik_solver_.ik_solve(des_gripper1_wrt_base)) 
	        {  nsolns++;
	           cout<<nphi<<",";
	           #cout<<":  found IK; nsolns = "<<nsolns<<endl;
	           gripper_affines_wrt_camera.push_back(affine_gripper_frame_wrt_camera_frame_);
	        }'''
	        nphi++
	        #else cout<<";  NO IK"<<endl;
	     
	    
	    print "\n"

	def simple_horiz_kvec_motion_psm2(self, O_needle, r_needle, kvec_yaw, gripper_affines_wrt_camera):
		#Eigen::Vector3d O_needle, float r_needle, float kvec_yaw, vector <Eigen::Affine3d> &gripper_affines_wrt_camera
		dphi = math.pi/40.0
	    #Eigen::Matrix3d R,R0;
	    #Eigen::Vector3d nvec,tvec,bvec,bvec0,tip_pos;
	    #Eigen::Affine3d des_gripper1_wrt_base;
	    bvec0=Vector(1,0,0)#at kvec_yaw=0, set needle axis parallel to camera-frame x-axis
	    #nvec<<-1,0,0;
	    #bvec<<1,0,0;
	    nvec=Vector(0,0,1)  #start w/ gripper x axis pointing "up" towards camera 
	    bvec = self.Rotz(kvec_yaw)*bvec0; #rotate the needle axis about camera z-axis
	    #cout<<"needle z-vec in camera frame: "<<bvec.transpose()<<endl;
	            
	    tvec = bvec*nvec;
	    R0 = Rotation(nvec,tvec,bvec)
	    R0 = R0.Inverse()
	    tip_pos=O_needle - r_needle*tvec
	    self.affine_gripper_frame_wrt_camera_frame_.p = tip_pos
	    self.affine_gripper_frame_wrt_camera_frame_.M = R0
	    #cout<<"start gripper1 orientation: "<<endl;
	    #cout<<affine_gripper_frame_wrt_camera_frame_.linear()<<endl;
	    #cout<<"needle tip: "<<affine_gripper_frame_wrt_camera_frame_.translation().transpose()<<endl;
	    del gripper_affines_wrt_camera[:]
	    nsolns = 0
	    nphi = 0
	    print "nphi: "
	    for phi in range(0.0,-math.pi,-dphi):
	        #need to rotate gripper frame about camera-frame x-axis;
	        # DO want to describe this as equiv rotation about gripper-frame z-axis
	        
	        #careful here: R0 is R_gripper/camera
	        # rotate about the camera-frame x-axis;
	        # more generally, rotate about k_vec, expressed in camera coords
	        #R = Rotx(phi)*R0; #Rot_k_phi(bvec, phi)*R0;
	        R = self.Rot_k_phi(bvec, phi)*R0
	        #cout<<"Rotx(phi):"<<endl;
	        #cout<<Rotx(phi)<<endl;
	        #cout<<"Rot_k_phi(bvec,phi):"<<endl;
	        #cout<<Rot_k_phi(bvec,phi)<<endl;
	        self.affine_gripper_frame_wrt_camera_frame_.M=R
	        R_Frame = posemath.toMatrix(self.affine_gripper_frame_wrt_camera_frame_)
	  		R_column2_numpy = R_Frame[0:3,1]
	  		R_column2 = Vector(R_column2_numpy[0],R_column2_numpy[1],R_column2_numpy[2])
	        tip_pos = O_needle - r_needle*R_column2 # assumes needle x-axis is parallel to gripper y axis
	        self.affine_gripper_frame_wrt_camera_frame_.p = tip_pos
	        #cout<<"gripper1 orientation at phi = "<<phi<<endl;
	        #cout<<affine_gripper_frame_wrt_camera_frame_.linear()<<endl;
	        #cout<<"needle tip: "<<affine_gripper_frame_wrt_camera_frame_.translation().transpose()<<endl;      
	        #gripper_affines_wrt_camera.push_back(affine_gripper_frame_wrt_camera_frame_);
	        #cout<<"enter 1: ";
	        #int ans;
	        #cin>>ans;
	        #express in psm base frame
	        des_gripper1_wrt_base = self.default_affine_lcamera_to_psm_two_.Inverse()*self.affine_gripper_frame_wrt_camera_frame_;
	        #try computing IK:
	        #cout<<"phi = "<<phi;
	        ''' ERDEM
	        if (ik_solver_.ik_solve(des_gripper1_wrt_base)) 
	        {  nsolns++;
	           cout<<nphi<<",";
	           #cout<<":  found IK; nsolns = "<<nsolns<<endl;
	           gripper_affines_wrt_camera.push_back(affine_gripper_frame_wrt_camera_frame_);
	        }'''
	        nphi++
	        #else cout<<";  NO IK"<<endl;
	     
	    
	    print "\n"

	    #test fnc--just computes a simple gripper path:
	def simple_test_gripper_motion(self, x, y, z, r, gripper_affines_wrt_camera):
		#float x, float y, float z, float r,vector <Eigen::Affine3d> &gripper_affines_wrt_camera

	def vers(self, phi): #float phi
	    return (1.0-cos(phi)) 
	    
	    #some utility functions:
	    #for rotations about z and y axes
    def Rotx(self, phi): #float phi
	    Rx_nrow = Vector(1.0, 0.0, 0.0)
	    Rx_trow = Vector(0.0, cos(phi), -sin(phi))
	    Rx_brow = Vector(0.0, sin(phi), cos(phi))

	    Rx = Rotation(Rx_nrow,Rx_trow,Rx_brow)

	    if(debug_needle_print):
	    	print "Rotx:" 
	    if(debug_needle_print):
	    	print Rx

	    return Rx  

    def Roty(self, phi): #float phi
    	Ry_nrow = Vector(cos(phi), 0.0, sin(phi))
	    Ry_trow = Vector(0.0, 1, 0.0)
	    Ry_brow = Vector(-sin(phi), 0, cos(phi))

	    Ry = Rotation(Ry_nrow,Ry_trow,Ry_brow)

	    if(debug_needle_print):
	    	print "Roty:" 
	    if(debug_needle_print):
	    	print Ry

	    return Ry  

    def Rotz(self, phi): #float phi
    	Rz_nrow = Vector(cos(phi), -sin(phi), 0.0)
	    Rz_trow = Vector(sin(phi), cos(phi), 0.0)
	    Rz_brow = Vector(0.0, 0.0, 1.0)

	    Rz = Rotation(Rz_nrow,Rz_trow,Rz_brow)

	    if(debug_needle_print):
	    	print "Rotz:" 
	    if(debug_needle_print):
	    	print Rz

	    return Rz  

    def Rot_k_phi(self, k_vec, phi): #Eigen::Vector3d k_vec,float phi
	    #Eigen::Matrix3d R_k_phi;
	    kx = k_vec[0]
	    ky = k_vec[1]
	    kz = k_vec[2]

    	K_nrow = Vector(0.0, -kz, ky)
	    K_trow = Vector(kz, 0.0, -kx)
	    K_brow = Vector(-ky, kx, 0.0)

	    K = Rotation(K_nrow,K_trow,K_brow)
	    I = Rotation()

	    R_k_phi = I + sin(phi)*K + (1-cos(phi))*K*K;
	    ''' minus-sign error in following:
	    R_k_phi(0,0) = kx*kx*vers(phi)+cos(phi);
	    R_k_phi(0,1) = ky*kx*vers(phi)-kz*sin(phi);
	    R_k_phi(1,0) = R_k_phi(0,1);
	    R_k_phi(0,2) = kz*kx*vers(phi)+ky*sin(phi);
	    R_k_phi(2,0) = R_k_phi(0,2);
	    R_k_phi(1,1) = ky*ky*vers(phi)+cos(phi);
	    R_k_phi(1,2) = kz*ky*vers(phi)-kx*sin(phi);
	    R_k_phi(2,1) = R_k_phi(1,2);
	    R_k_phi(2,2) = kz*kz*vers(phi)+cos(phi);
	     '''
	    return R_k_phi;
	
	def print_affine(self, affine): #print out an affine for debug
	#Eigen::Affine3d affine
		print "Rotation: "
    	print affine.M
        print "origin: "affine.p;

	def write_needle_drive_affines_to_file(self, gripper_affines_wrt_camera):

	def write_psm2_needle_drive_affines_to_file(self, gripper_affine_psm1, squeeze_cmd, psm2_gripper_affines_wrt_camera):
    #Eigen::Affine3d gripper_affine_psm1, float squeeze_cmd, vector <Eigen::Affine3d> &psm2_gripper_affines_wrt_camera
	    
