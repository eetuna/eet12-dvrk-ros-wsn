/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen
  Created on: 2013-07-14

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights
  Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// Brief: da Vinci PSM kinematics

#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

#include <cisstRobot/robManipulator.h>
#include <cisstVector.h>

#include <cisst_ros_bridge/mtsCISSTToROS.h>
#include <cisst_ros_bridge/mtsROSToCISST.h>

#include "dvrk_kinematics/psm_logic.h"

using namespace std;
// set up joint state variables
static vctDoubleVec psm_joint_current, psm_joint_currentTest;
static vctDoubleVec psm_joint_command, psm_joint_currentIK;
static vctDoubleVec psm_joint_command_prev;
static vctFrm4x4 psm_pose_current;
static vctFrm4x4 psm_pose_command;
static double mtm_gripper;
static int control_mode;


enum { NUM_ROWS=6, NUM_COLS=6};
vctDynamicMatrix<double> JacobMat(NUM_ROWS, NUM_COLS, 0.0 );

int main(int argc, char** argv)
{
    
     // --- cisst robManipulator ---
    string filename = ros::package::getPath("dvrk_robot");
    filename.append("/config/dvpsm.rob");
    robManipulator psm_manip;
    robManipulator::Errno result;
    result = psm_manip.LoadRobot(filename);



    // --- initialize joint current/command -----
    psm_joint_current.SetSize(6);  // 6 + 1 (open angle)
    psm_joint_current.SetAll(0.0);

    psm_joint_currentIK.SetSize(6);  // 6 + 1 (open angle)
    psm_joint_currentIK.SetAll(0.0);


    psm_joint_command.ForceAssign(psm_joint_current);

   
    vctFrm4x4 frame6to7;
    frame6to7.Assign(0.0, -1.0, 0.0, 0.0,
                     0.0,  0.0, 1.0, 0.0102,
                     -1.0, 0.0, 0.0, 0.0,
                     0.0,  0.0, 0.0, 1.0);


    // ------------ run() --------------------------
   
         cout << result << endl;

        vctFrm4x4 pose6, pose7, psm_pose0to6, psm_pose0to7, psm_pose0to7_GivenPose;
        // --------- Compute current pose & publish ----------
        // psm forward kinematics'
        psm_joint_current[2] = 0.0;
        psm_pose0to6 = psm_manip.ForwardKinematics(psm_joint_current);



        cout << "Home position" << endl;
        cout << "FK fram0to6:" << endl;
        cout << psm_pose0to6 << endl;

        // publish current pose

        // ---------- Compute command psm joint positin -----------

        psm_pose0to7 = psm_pose0to6 * frame6to7;

        cout << "FK result:" << endl;
        cout << psm_pose0to7 << endl;


        psm_joint_current[0]  = -0.23;
        psm_joint_current[1]  =  -0.31;
        psm_joint_current[2]  = 0.1;
        psm_joint_current[3]  = -0.14;
        psm_joint_current[4]  = 1.19;
        psm_joint_current[5]  = 1.11;
        psm_joint_current[6]  = 0;

        psm_pose0to6 = psm_manip.ForwardKinematics(psm_joint_current);

        cout << "using joints: " << psm_joint_current << endl;
        cout << "FK fram0to6:" << endl;
        cout << psm_pose0to6 << endl;
        psm_pose0to7 = psm_pose0to6 * frame6to7;
        cout << "FK result:" << endl;
        cout << psm_pose0to7 << endl;


        psm_pose0to7_GivenPose.Assign( -0.2265,    0.4496,    0.8641,   -0.0097,
                             0.6337,    0.7417,  -0.2198,   0.0166,
                            -0.7397,    0.4978,   -0.4529,   -0.0888,
                             0,         0,         0,    1.0000);


        psm_pose0to6 = psm_pose0to7 *  frame6to7.InverseSelf();

        cout << "InverseTest" << endl;
        cout << "FK result:" << endl;
        cout << psm_pose0to7 << endl;
        cout << "FK fram0to6:" << endl;
        cout << psm_pose0to6 << endl;


        double tolerance=1e-12;
        size_t Niteration=1000;

        //psm_manip.InverseKinematics(psm_joint_currentIK, psm_pose0to6, tolerance, Niteration);
        psm_manip.InverseKinematics(psm_joint_currentIK, psm_pose0to6, tolerance, Niteration);

        cout << "original input joints: " << endl;
        cout << psm_joint_current << endl;

        cout << "IK result joints " << endl;
        cout << psm_joint_currentIK << endl;

        cout << "IK error: "<< endl;
        cout << psm_joint_current - psm_joint_currentIK << endl;

        bool TestJacobian;


        //psm_joint_currentTest[0] = psm_joint_current[0];
        //psm_joint_currentTest[1] = psm_joint_current[1];
        //psm_joint_currentTest[2] = psm_joint_current[2];
        //psm_joint_currentTest[3] = psm_joint_current[3];
        //psm_joint_currentTest[4] = psm_joint_current[4];
        //psm_joint_currentTest[5] = psm_joint_current[5];
        
        //cout << "jacobian joints test" << endl;
        //cout << psm_joint_currentTest  << endl;


        /*
        for (int i = 0; i < NUM_ROWS; i++) {
            for (int j = 0; j < NUM_COLS; j++){
                myMatrix.at(i,j) = 0.0;
            }
        }*/
        cout << "myMatrix" << JacobMat << endl;
        
        
        TestJacobian  = psm_manip.JacobianBody(psm_joint_current,JacobMat);

        

        cout << "Jacobian Body Matrix: "<< endl;
        cout << JacobMat << endl;
        cout << "Jacobian bool result:" << TestJacobian << endl;
        


        // MODE_RESET: send HOME joint position
        // MODE_MANUAL: controled by JSP GUI, not sending anything to jsp
        // MODE_HOLD: disable JSP GUI, not sending anything to jsp
        // MODE_TELEOP: take command pose, send to jsp


        // do nothing for MANUAL
        // controlled using Slifer GUI
        /*
        if (control_mode != PSM::MODE_MANUAL && control_mode != PSM::MODE_HOLD) {
            pub_psm_joint_state_cmd.publish(msg_js);
        }
        */



    return 0;
}
