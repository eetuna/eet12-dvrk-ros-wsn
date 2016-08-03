from robot import *
import PyKDL
#create a square of length `length`
def home(robotName):
    r = robot(robotName)
    
    r.move_cartesian_translation([0.0,0.0,-0.1])
    r.move_cartesian_translation([0.1,0.0,-0.1])
    r.move_cartesian_translation([0.05,0.05,-0.05],True)
    r.delta_move_cartesian_translation([0.06,0.06,-0.06],True)
    r.delta_move_cartesian_translation([0.04,0.08,-0.04],True)
    #the length of the square
    '''
    input_length = raw_input('Enter the length of the square : ')
    length = float(input_length)
    #move in cartesian space
    r.move_cartesian_translation([0.0,length, -0.15])
    r.move_cartesian_translation([length, length, -0.15])
    r.move_cartesian([length, 0.0, -0.15])
    r.move_cartesian([0.0, 0.0, -0.15])'''


if __name__ == '__main__':
    if (len(sys.argv) != 2):
        print sys.argv[0] + ' requires one argument, i.e. name of dVRK arm'
    else:
        home(sys.argv[1])