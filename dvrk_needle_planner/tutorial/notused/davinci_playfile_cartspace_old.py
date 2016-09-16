from robot import *
from needle_planner import *
import rospy


def cartesian_move(robotName,read_playfile):
	r = robot(robotName)

	rospy.init_node('playfile_cartspace_test')

	infile = open(read_playfile,'r')
	infile.read()


	#print infile
	infile.close()
	



if __name__ == '__main__':
    """Here will check if we are given the name of the robot arm or not, if we are we run cartesian_move()"""
    if (len(sys.argv) != 3):
        print sys.argv[0] + ' requires two arguments, i.e. name of dVRK arm and .csp file'
    else:
        cartesian_move(sys.argv[1],sys.argv[2])
