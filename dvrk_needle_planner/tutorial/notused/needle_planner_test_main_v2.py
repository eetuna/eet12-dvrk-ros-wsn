import rospy

from robot import *
from needle_planner import *
from tf import transformations
from tf_conversions import posemath

rospy.init_node('needle_planner_test_main')

tissue_normal = Vector(0,0,-1)
entrance_pt = Vector(-0.1,0.05,0.1)
exit_pt = Vector(-0.09,0.05,0.1)

rospy.loginfo("main: instantiating an object of type NeedlePlanner")
needlePlanner = needle_planner()

needlePlanner.compute_tissue_frame_wrt_camera(entrance_pt, exit_pt, tissue_normal)
gripper_affines_wrt_camera = []
#needlePlanner.compute_needle_drive_gripper_affines(gripper_affines_wrt_camera)


rospy.loginfo("getting transforms from camera to PSMs")
tfListener = tf.TransformListener()
ntries = 0
tferr = bool('')
while tferr:
	if (ntries > 5):
		break:
	try:
		(trans_one,rot_one) = tflistener.lookupTransform('left_camera_optical_frame', 'one_psm_base_link', rospy.Time(0))
		(trans_two,rot_two) = tflistener.lookupTransform('left_camera_optical_frame', 'two_psm_base_link', rospy.Time(0))
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		tferr = True
		rospy.sleep(rospy.Duration(0.5))
		ntries += 1

if terr:
	affine_lcamera_to_psm_one = Frame()
	affine_lcamera_to_psm_one.p = Vector(-0.155, -0.03265, 0.0)
	nvec = Vector(-1,0,0)
	tvec = Vector(0,1,0)
	bvec = Vector(0.0,-1)
	R = Rotation(nvec,tvec,bvec)
	R = R.Inverse()
	affine_lcamera_to_psm_one.M = R

	affine_lcamera_to_psm_two = Frame()
	affine_lcamera_to_psm_two.p = Vector(0.145,-0.03265,0.0)
	affine_lcamera_to_psm_two.R = R

	rospy.logwarn("using default transform")
else:
	rospy.loginfo("tf is good")
	affine_lcamera_to_psm_one = posemath.fromTf(trans_one,rot_one)
	affine_lcamera_to_psm_two = posemath.fromTf(trans_two,rot_two)

rospy.loginfo("transform from left camera to psm one:")
print affine_lcamera_to_psm_one
rospy.loginfo("transform from left camera to psm two:")
print affine_lcamera_to_psm_two

while not rospy.is_shutdown(): 
