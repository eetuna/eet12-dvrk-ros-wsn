"""In this example we will take a look at the how to move in joint space. Please note we can move a different distance than the one specified. We will show how the following methods world:
 * delta_move_joint_translation()
 * delta_move_joint_rotation
 * move_joint_translation
 * move_joint_rotation

Lets take a look:"""

from robot import *
import time

def joint_move(robotName):
    """Here is where we initialize the robot, in this demo we called the robot r, and we move the robot in joint space accordingly.

    :param robotName: the name of the robot used """
    r = robot(robotName)
#look at size
    print 'After, move to position [0,0,0.56,0,0,0,0]:'
    r.move_joint_list([0.0, 0.0, 0.0060, 0.0, 0.0, 0.0, 0.0], interpolate = False)
    print r.get_desired_joint_position()
    '''
    print 'After, move to position [0,0,0.2,0,0,0,0]:'
    r.move_joint_list([0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0], interpolate = True)
    print r.get_desired_joint_position()
   
    print 'After, move to position [0,0,0.4,0,0,0,0]:'
    r.move_joint_list([0.0, 0.0, 0.004, 0.0, 0.0, 0.0, 0.0], interpolate = True)
    print r.get_desired_joint_position()

    r.move_joint_list([0.1, 0.0, 0.004, 0.0, 0.0, 0.0, 0.0], interpolate = True)
    print r.get_desired_joint_position()'''

if __name__ == '__main__':
    """Here will check if we are given the name of the robot arm or not, if we are we run joint_move()"""
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        joint_move(sys.argv[1])
