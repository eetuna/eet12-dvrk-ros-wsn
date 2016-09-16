from robot import *
import time

def dvrk_test(robotName):

	r = robot(robotName)


if __name__ == '__main__':
    """Here will check if we are given the name of the robot arm or not, if we are we run joint_move()"""
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        dvrk_test(sys.argv[1])
