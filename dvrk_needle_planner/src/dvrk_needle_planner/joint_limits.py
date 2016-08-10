import math
import numpy as np
from copy import *
# NEED TO FIND THESE:
DH_q_max0 = 1.0 #deg2rad*45 #141 #51
DH_q_max1 = 0.7 #deg2rad*45
DH_q_max2 = 0.23 #0.5
DH_q_max3 = 2.25 #deg2rad*180
DH_q_max4 = 1.57 #deg2rad*90
DH_q_max5 = 1.39 #deg2rad*90 #
DH_q_max6 = 1.57 #deg2rad*90

#-141, -123, -173.5, -3, -175.25, -90, -175.25
DH_q_min0 = -1.0 #-deg2rad*45 #51 #141
DH_q_min1 = -0.7 #-deg2rad*45
DH_q_min2 =  0.01 
DH_q_min3 = -2.25 #-deg2rad*180
DH_q_min4 = -1.57 #-deg2rad*90
DH_q_min5 = -1.39 #-deg2rad*90 #
DH_q_min6 = -1.57 #-deg2rad*90


q_lower_limits = np.array([DH_q_min0,DH_q_min1, DH_q_min2, DH_q_min3, DH_q_min4, DH_q_min5, DH_q_min6])
q_upper_limits = np.array([DH_q_max0,DH_q_max1, DH_q_max2, DH_q_max3, DH_q_max4, DH_q_max5, DH_q_max6])


debug_needle_print = True
"""
def fit_q_to_range(q_min, q_max, q):
    while (q<q_min):
        print "q min before", q
        q+= 2.0*math.pi
        print "q min after", q
    
    while (q>q_max):
        print "q max before", q
        q -= 2.0*math.pi
        print "q max after", q
    
    if (q<q_min):
        return False #rtn false if no periodic soln in range
    else:
        return True

"""


"""
    def fit_q_to_range(self, q_min, q_max, q):
        self.qRange = deepcopy(q)
        while (self.qRange<q_min):
            print "q min before", self.qRange
            self.qRange+= 2.0*math.pi
            print "q min after", self.qRange
        
        while (self.qRange>q_max):
            print "q max before", self.qRange
            self.qRange-= 2.0*math.pi
            print "q max after", self.qRange
        
        if (self.qRange<q_min):
            q = self.qRange
            return False #rtn false if no periodic soln in range
        else:
            q = self.qRange
            return True



    def fit_joints_to_range(self, qvec):
        self.qvecIK = qvec
        fits = True
        does_fit = False
        for i in range(6):
            #print "qvec", qvec
            #q = qvec[i]
            q = self.qvecIK[i]
            #treat d3 differently
            if (i!=2): 
                print "q before: ", q
                does_fit = self.fit_q_to_range(q_lower_limits[i],q_upper_limits[i],q)
                print "does_fit: ", does_fit
                print "q after: ", q
            #special case for d3...although generic formula also works in this case
            else: 
                does_fit = True
                if (q<q_lower_limits[i]): 
                    does_fit = False
                if (q>q_upper_limits[i]): 
                    does_fit = False
            
            if (not does_fit): 
                if(debug_needle_print):
                    print("IK err: jnt %d  lower lim: %f upper lim: %f desired val = %f"%(i,q_lower_limits[i],q_upper_limits[i],q))
            
            #qvec[i] = q
            self.qvecIK[i] = q
            fits = fits and does_fit
        
        if (fits):
            return True
        else:
            return False
"""

class joint_test:
    def __init__(self):
        self.qvecIK = np.array([0, 0, 0, 0, 0, 0])
        self.qRange = np.array([0, 0, 0, 0, 0, 0])

    def fit_q_to_range(self, q_min, q_max,i):
        while (self.qvecIK[i]<q_min):
            print "q min before", self.qvecIK[i]
            self.qvecIK[i] += 2.0*math.pi
            print "q min after", self.qvecIK[i]
        
        while (self.qvecIK[i]>q_max):
            print "q max before", self.qvecIK[i]
            self.qvecIK[i] -= 2.0*math.pi
            print "q max after", self.qvecIK[i]
        
        if (self.qvecIK[i]<q_min):
            print self.qvecIK[i]
            return False #rtn false if no periodic soln in range
        else:
            print self.qvecIK[i]
            return True



    def fit_joints_to_range(self, qvec):
        self.qvecIK = qvec
        fits = True
        does_fit = False
        for i in range(6):
            #print "qvec", qvec
            #q = qvec[i]
            #q = self.qvecIK[i]
            #treat d3 differently
            if (i!=2): 
                print "q before: ", self.qvecIK[i]
                does_fit = self.fit_q_to_range(q_lower_limits[i],q_upper_limits[i],i)
                print "does_fit: ", does_fit
                print "q after: ", self.qvecIK[i]
            #special case for d3...although generic formula also works in this case
            else: 
                does_fit = True
                if (self.qvecIK[i]<q_lower_limits[i]): 
                    does_fit = False
                if (self.qvecIK[i]>q_upper_limits[i]): 
                    does_fit = False
            
            if (not does_fit): 
                if(debug_needle_print):
                    print("IK err: jnt %d  lower lim: %f upper lim: %f desired val = %f"%(i,q_lower_limits[i],q_upper_limits[i],self.qvecIK[i]))
            
            #qvec[i] = q
            # self.qvecIK[i] = q
            fits = fits and does_fit
        
        if (fits):
            return True
        else:
            return False



if __name__ == '__main__':

    qvecIK = np.array([1.2, 2, 0.15, 2.5, 3.5, 7.2])
    jointTest = joint_test()
    jointTest.fit_joints_to_range(qvecIK)
