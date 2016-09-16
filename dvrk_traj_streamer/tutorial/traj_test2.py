from robot import *
import rospy



def test_initialize(robotName):
    """Here is where we initialize the robot, in this demo we called the robot r, and show how to use each methods.
	
    :param robotName: the name of the robot used """
    global r1
    r1 = robot(robotName)
    r1.home()
    print r1.get_current_joint_position()
    rospy.spin()

if __name__ == '__main__':
    """Here will check if we are given the name of the robot arm or not, if we are we run getters()."""
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        test_initialize(sys.argv[1])
