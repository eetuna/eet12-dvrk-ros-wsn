from robot import *
import rospy

global robot1

def test_init():
    global r1, r2
    r1 = robot('PSM1')
    r2 = robot('PSM2')

    r1.home()
    r2.home()

    return r1, r2

