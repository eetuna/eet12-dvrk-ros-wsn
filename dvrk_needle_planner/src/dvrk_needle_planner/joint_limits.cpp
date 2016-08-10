
// NEED TO FIND THESE:
const double DH_q_max0 = 1.0; //deg2rad*45; //141; //51;
const double DH_q_max1 = 0.7; //deg2rad*45;
const double DH_q_max2 = 0.23; //0.5;
const double DH_q_max3 = 2.25; //deg2rad*180;
const double DH_q_max4 = 1.57; //deg2rad*90;
const double DH_q_max5 = 1.39; //deg2rad*90; //
const double DH_q_max6 = 1.57; //deg2rad*90;

//-141, -123, -173.5, -3, -175.25, -90, -175.25
const double DH_q_min0 = -1.0; //-deg2rad*45; //51; //141;
const double DH_q_min1 = -0.7; //-deg2rad*45;
const double DH_q_min2 =  0.01; 
const double DH_q_min3 = -2.25; //-deg2rad*180;
const double DH_q_min4 = -1.57; //-deg2rad*90;
const double DH_q_min5 = -1.39; //-deg2rad*90; //
const double DH_q_min6 = -1.57; //-deg2rad*90;

const double DH_a_params[7]={DH_a1,DH_a2,DH_a3,DH_a4,DH_a5,DH_a6,DH_a7};
double DH_d_params[7] = {DH_d1, DH_d2, DH_d3, DH_d4, DH_d5, DH_d6,DH_d7};
const double DH_alpha_params[7] = {DH_alpha1, DH_alpha2, DH_alpha3, DH_alpha4, DH_alpha5, DH_alpha6,DH_alpha7};
const double DH_q_offsets[7] = {DH_q_offset0,DH_q_offset1, DH_q_offset2, DH_q_offset3, DH_q_offset4, DH_q_offset5, DH_q_offset6};
const double q_lower_limits[7] = {DH_q_min0,DH_q_min1, DH_q_min2, DH_q_min3, DH_q_min4, DH_q_min5, DH_q_min6};
const double q_upper_limits[7] = {DH_q_max0,DH_q_max1, DH_q_max2, DH_q_max3, DH_q_max4, DH_q_max5, DH_q_max6};

bool Davinci_IK_solver::fit_q_to_range(double q_min, double q_max, double &q) {
    while (q<q_min) {
        q+= 2.0*M_PI;
    }
    while (q>q_max) {
        q-= 2.0*M_PI;
    }    
    if (q<q_min)
        return false; //rtn false if no periodic soln in range
    else
        return true;
}

bool Davinci_IK_solver::fit_joints_to_range(Vectorq7x1 &qvec) {
    bool fits=true;
    bool does_fit;
    double q;
    for (int i=0;i<7;i++) {
        q = qvec[i];
        if (i!=2) { //treat d3 differently
                does_fit = fit_q_to_range(q_lower_limits[i],q_upper_limits[i],q);
        }
        else { //special case for d3...although generic formula also works in this case
            does_fit=true;
            if (q<q_lower_limits[i]) does_fit=false;
            if (q>q_upper_limits[i]) does_fit=false;
        }
        if (!does_fit) {
            if(debug_print) ROS_WARN("IK err: jnt %d;  lower lim: %f; upper lim: %f desired val = %f;",
                    i,q_lower_limits[i],q_upper_limits[i],q);
        }
        qvec[i] = q;
        fits = fits&&does_fit;
    }
    if (fits)
        return true;
    else
        return false;
}