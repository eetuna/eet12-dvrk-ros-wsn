from robot import *
from needle_planner import *
import rospy


rospy.init_node('needle_planner_test_main')

tissue_normal = Vector(0,0,-1)
entrance_pt = Vector(-0.1,0.05,0.1)
exit_pt = Vector(-0.09,0.05,0.1)

rospy.loginfo("main: instantiating an object of type NeedlePlanner")
needlePlanner = needle_planner()

needlePlanner.compute_tissue_frame_wrt_camera(entrance_pt, exit_pt, tissue_normal)