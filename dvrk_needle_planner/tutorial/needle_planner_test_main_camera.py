from robot import *
from needle_planner import *

#import rospy
#rospy.init_node('needle_planner_test_main')

tissue_normal = Vector(0,0,-1)
entrance_pt = Vector(0.0,0.1,0.1)
exit_pt = Vector(0.01,0.1,0.1)

rospy.loginfo("main: instantiating an object of type NeedlePlanner")
needlePlanner = needle_planner()

gripper_affines_wrt_camera = []
gripper_affines_wrt_psm = []
psm2_gripper_affines_wrt_camera = []
psm2_gripper_affines_psm = []
#print gripper_affines_wrt_camera
#print psm2_gripper_affines_wrt_camera

tilt = float(raw_input('Enter tilt of needle z-axis w/rt tissue: ')) #in between 0.5 to -0.5 radians
needlePlanner.set_psi_needle_axis_tilt_wrt_tissue(tilt)

needlePlanner.compute_tissue_frame_wrt_camera(entrance_pt, exit_pt, tissue_normal)

#print "enter tilt of needle z-axis w/rt tissue: "


needlePlanner.compute_needle_drive_gripper_affines(gripper_affines_wrt_camera)
nposes = len(gripper_affines_wrt_camera)
for i in range(nposes):
	print("pose %d"%i)
	print gripper_affines_wrt_camera[i]
#needlePlanner.compute_needle_drive_gripper_affines(psm2_gripper_affines_wrt_camera)
#print gripper_affines_wrt_camera
#print psm2_gripper_affines_wrt_camera

needlePlanner.write_needle_drive_affines_to_file(gripper_affines_wrt_camera)
#needlePlanner.write_psm2_needle_drive_affines_to_file(psm2_gripper_affines_wrt_camera)