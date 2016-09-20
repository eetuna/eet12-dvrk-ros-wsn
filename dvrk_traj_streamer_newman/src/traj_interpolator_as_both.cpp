// wsn pgm to receive Davinci trajectories and interpolate them smoothly
// as commands to Davinci;
// starting w/ just 4DOF (no gripper)
#include <davinci_traj_streamer/davinci_traj_streamer.h>
//#include <davinci_kinematics/davinci_joint_publisher.h>
#include <actionlib/server/simple_action_server.h>

#include <eigen_conversions/eigen_msg.h>

//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../baxter_traj_streamer/action/traj.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (traj) and appended name (Action)
#include<davinci_traj_streamer/trajAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>

#include <iostream>
#include <string>
#include <fstream>




#include <chrono>
#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>
//#include <message_instance.h>
/* maybe restore this later
bool trajInterpStatusSvc(cwru_srv::simple_bool_service_messageRequest& request, cwru_srv::simple_bool_service_messageResponse& response) {
    //ROS_INFO("responding to service request: status of trajectory interpolator");
    response.resp = working_on_trajectory; // return status of "working on trajectory"
    return true;
}
*/

using namespace std;

class TrajActionServer {
private:

    ros::NodeHandle nh_; // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in baxter_traj_streamer/action/traj.action
    // the type "trajAction" is auto-generated from our name "traj" and generic name "Action"
    actionlib::SimpleActionServer<davinci_traj_streamer::trajAction> as_;

    // here are some message types to communicate with our client(s)
    davinci_traj_streamer::trajGoal goal_; // goal message, received from client
    davinci_traj_streamer::trajResult result_; // put results here, to be sent back to the client when done w/ goal
    davinci_traj_streamer::trajFeedback feedback_; // not used in this example; 
    // would need to use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    //baxter_core_msgs::JointCommand right_cmd, left_cmd;
    trajectory_msgs::JointTrajectory new_trajectory; // member var to receive new traj's;
    int g_count; //=0; //just for testing
    bool working_on_trajectory; // = false;
    void command_joints(Eigen::VectorXd q_cmd);

    void dvrk_set_state(std_msgs::String state);
    //string robot_state, robot_state_pre;


    pair<string, string> PSMstates;
    pair<string, string> PSMstates_pre;

    string robot_state1 = "DVRK_UNINITIALIZED";
    string robot_state_pre1 = robot_state1;

    string robot_state2 = "DVRK_UNINITIALIZED";
    string robot_state_pre2 = robot_state2;

    PSMstates.first = robot_state1;
    PSMstates.second = robot_state2;

    PSMstates_pre.first =  robot_state_pre1;
    PSMstates_pre.second = robot_state_pre2;
    //ros::Publisher  j1_pub_,j2_pub_,j2_1_pub_,j2_2_pub_,j2_3_pub_,j2_4_pub_,j2_5_pub_,j3_pub_,j4_pub_,j5_pub_,j6_pub_,j7_pub_;

    ros::Publisher set_robot_state_publisher_PSM1,set_position_joint_publisher_PSM1,set_robot_state_publisher_PSM2,set_position_joint_publisher_PSM2;

    //set_position_goal_joint_publisher,
    //set_position_cartesian_publisher, set_position_goal_cartesian_publisher, set_jaw_position_publisher, set_wrench_body_publisher, set_wrench_spatial_publisher;

    ros::Subscriber get_robot_state_subscriber_PSM1, get_state_joint_desired_subsriber_PSM1,  get_robot_state_subscriber_PSM2, get_state_joint_desired_subsriber_PSM2;
    //get_position_cartesian_desired_suscriber,
    //get_state_joint_current_subscriber, get_position_cartesian_current_subscriber, get_goal_reached_subsriber,;

    void initializePublishers();
    void initializeSubscribers();
    
    
    
    void robot_state_callback_PSM1(const std_msgs::String& strc);
    void robot_state_callback_PSM2(const std_msgs::String& strc);
    
    Eigen::VectorXd get_joint_error(Eigen::VectorXd qvec_desired);

    //void goal_reached_callback(const std_msgs::Bool& boolc);


    //void state_joint_desired_callback(const sensor_msgs::JointState& jointState);


    //void position_cartesian_desired_callback(const geometry_msgs::Pose& pose);


    void state_joint_current_callback_PSM1(const sensor_msgs::JointState& jointState);
    void state_joint_current_callback_PSM2(const sensor_msgs::JointState& jointState);

    std::pair<Eigen::VectorXd, Eigen::VectorXd> get_joint_current();

    ofstream myfile;
    //void position_cartesian_current_callback(const geometry_msgs::Pose& pose);


    //Eigen::Affine3d position_cartesian_desired, position_cartesian_current;
    //Eigen::VectorXd position_joint_current_PSM1, position_joint_desired_PSM, position_joint_current_PSM2, position_joint_desired_PSM12;

    std::pair<Eigen::VectorXd, Eigen::VectorXd> position_joint_current;
    std::pair<Eigen::VectorXd, Eigen::VectorXd> position_joint_desired;

    ///std_msgs::String robot_state;
    bool goal_reached;
    bool g_joint_current_got_callback=false;
    bool g_cartesian_desired_got_callback=false;
    bool g_joint_desired_got_callback=false;
    bool g_cartesian_current_got_callback=false;


    //DavinciJointPublisher davinciJointPublisher; //(&nh_); //:&nh_;//(&nh_);//(nh_); //DavinciJointPublisher davinciJointPublisher(&nh);
public:
    //TrajActionServer(ros::NodeHandle nh);
    TrajActionServer(ros::NodeHandle &nh); //define the body of the constructor outside of class definition

    ~TrajActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<davinci_traj_streamer::trajAction>::GoalConstPtr& goal);
    bool update_trajectory(double traj_clock, trajectory_msgs::JointTrajectory trajectory, Eigen::VectorXd qvec_prev, 
        int &isegment, Eigen::VectorXd &qvec_new);
    void home();

    pair<string, string> get_robot_state();
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, which is a member method
// of our class exampleActionServer.  Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB takes one argument
// the final argument  "false" says don't start the server yet.  (We'll do this in the constructor)

//TrajActionServer::TrajActionServer(ros::NodeHandle nh) :nh_(nh),
TrajActionServer::TrajActionServer(ros::NodeHandle &nh) :nh_(nh),
as_(nh, "trajActionServer", boost::bind(&TrajActionServer::executeCB, this, _1), false)
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of TrajActionServer...");
    cout << "in constructor" << endl;
    cout << "robot_state_PSM1: " << robot_state1 << endl;
    cout << "robot_state_pre_PSM1: " << robot_state_pre1 << endl;
    cout << "robot_state_PSM2: " << robot_state2 << endl;
    cout << "robot_state_pre_PSM2: " << robot_state_pre2 << endl;
    initializePublishers();
    //ros::Duration(0.1).sleep();
    //m_bNewState = false;
    initializeSubscribers();

    std_msgs::String state1, state2;
    state1.data = "DVRK_UNINITIALIZED";
    state2.data = "DVRK_UNINITIALIZED";
    dvrk_set_state(state1, state2);

    g_count = 0;
    working_on_trajectory = false;
    ROS_INFO("starting action server: trajActionServer ");


    


    as_.start(); //start the server running

}




void TrajActionServer::initializePublishers() {
   

        set_robot_state_publisher_PSM1 = nh_.advertise<std_msgs::String>("/dvrk/PSM1/set_robot_state", 1, true);
        set_robot_state_publisher_PSM2 = nh_.advertise<std_msgs::String>("/dvrk/PSM2/set_robot_state", 1, true);
        

        set_position_joint_publisher_PSM1 = nh_.advertise<sensor_msgs::JointState>("/dvrk/PSM1/set_position_joint", 1, true);
        set_position_joint_publisher_PSM2 = nh_.advertise<sensor_msgs::JointState>("/dvrk/PSM2/set_position_joint", 1, true);


}

void TrajActionServer::initializeSubscribers() {


    get_robot_state_subscriber_PSM1 = nh_.subscribe("/dvrk/PSM1/robot_state", 1000, &TrajActionServer::robot_state_callback_PSM1, this); 
    get_robot_state_subscriber_PSM2 = nh_.subscribe("/dvrk/PSM2/robot_state", 1000, &TrajActionServer::robot_state_callback_PSM2, this); 


    get_state_joint_current_subscriber_PSM1 = nh_.subscribe("/dvrk/PSM1/state_joint_current", 1000, &TrajActionServer::state_joint_current_callback_PSM1, this);
    get_state_joint_current_subscriber_PSM2 = nh_.subscribe("/dvrk/PSM2/state_joint_current", 1000, &TrajActionServer::state_joint_current_callback_PSM2, this);
    //get_robot_state_subscriber = nh_.subscribe(full_ros_namespace + "/position_cartesian_current", 100, position_cartesian_current_callback);



}

void TrajActionServer::dvrk_set_state(std_msgs::String state1, std_msgs::String state2){
        /*"""Simple set state with block.

        :param state: the robot state
        :param timeout: the lenghth you want to wait for robot to change state
        :return: whether or not the robot state has been successfuly set
        :rtype: Bool"""*/

        set_robot_state_publisher_PSM1.publish(state1);
        set_robot_state_publisher_PSM2.publish(state2);
}
void TrajActionServer::home()
{
/*
        """This method will provide power to the robot as will as home
        the robot. This method requries the robot name."""*/


        std_msgs::String state1, state2;
        state1.data = "Home"
        state2.data = "Home";
        dvrk_set_state(state1, state2);

        pair<string, string> PSMstates_current = get_robot_state();

        //cout << "robot_state_PSM1: " << PSMstates.PSM1 << endl;
        //cout << "robot_state_PSM2: " << PSMstates.PSM2 << endl;
        //ros::Duration(0.1).sleep();
        cout << "dvrk set state home done" << endl;

        ROS_INFO("homing complete");

        cout << "robot_state_PSM1: " << PSMstates_current.PSM1 << endl;
        cout << "robot_state_PSM2: " << PSMstates_current.PSM2 << endl;

        //cout << "robot_state_PSM1: " << get_robot_state() << endl;
        //cout << "robot_state_PSM2: " << get_robot_state() << endl;
        //rospy.loginfo(rospy.get_caller_id() + ' <- homing complete')

}



pair<string, string> TrajActionServer::get_robot_state()
{
    //nh_.subscribe("/dvrk/PSM1/robot_state", 1, &TrajActionServer::robot_state_callback, this);
    cout << "robot_state_PSM1: " << PSMstates.first << endl;
    cout << "robot_state_PSM2: " << PSMstates.second << endl;
    //ROS_INFO("-> current state is %s", robot_state);
    return PSMstates;

}

void TrajActionServer::robot_state_callback_PSM1(const std_msgs::String& strc)
{
  PSMstates_pre.first = PSMstates.first;
  ROS_INFO("-> PSM1 current state is %s", strc.data.c_str());
  PSMstates.first = strc.data.c_str();
}

void TrajActionServer::robot_state_callback_PSM2(const std_msgs::String& strc)
{
  PSMstates_pre.second = PSMstates.second;
  ROS_INFO("-> PSM2 current state is %s", strc.data.c_str());
  PSMstates.second = strc.data.c_str();
}

void TrajActionServer::state_joint_current_callback_PSM1(const sensor_msgs::JointState& jointState)
{
  //ROS_INFO(getCallerID() + " -> current state is %s", data.data);
    //Eigen::VectorXd position_joint_desired;
        position_joint_current.first.resize(7);

        for (int i=0;i<7;i++)
        {
            position_joint_current.first[i] = jointState.position[i];
        }


}

void TrajActionServer::state_joint_current_callback_PSM2(const sensor_msgs::JointState& jointState)
{
  //ROS_INFO(getCallerID() + " -> current state is %s", data.data);
    //Eigen::VectorXd position_joint_desired;
        position_joint_current.second.resize(7);

        for (int i=0;i<7;i++)
        {
            position_joint_current.second[i] = jointState.position[i];
        }


}

std::pair<Eigen::VectorXd, Eigen::VectorXd> TrajActionServer::get_joint_current()
{

    return position_joint_current;
}

Eigen::VectorXd TrajActionServer::get_joint_error(Eigen::VectorXd qvec_desired)
{
  //ROS_INFO(getCallerID() + " -> current state is %s", data.data);
    //Eigen::VectorXd position_joint_desired;
        Eigen::VectorXd position_joint_error;
        position_joint_error.resize(14);

        for (int i=0;i<7;i++)
        {
            position_joint_error[i] = position_joint_current.first[i] - qvec_desired[i];
            position_joint_error[i+7] = position_joint_current.second[i] - qvec_desired[i+7];
        }

        return position_joint_error;

}

void TrajActionServer::command_joints(Eigen::VectorXd q_cmd) {
    std_msgs::String state1, state2;
    state1.data = "DVRK_POSITION_JOINT";
    state2.data = "DVRK_POSITION_JOINT";
    dvrk_set_state(state1, state2);
    pair<string, string> PSMstates_current = get_robot_state();
    
            cout << "robot_state_PSM1: " << PSMstates_current.PSM1 << endl;
            cout << "robot_state_PSM2: " << PSMstates_current.PSM2 << endl;

    sensor_msgs::JointState jointState1, jointState2;

    jointState1.position.resize(7);
    jointState2.position.resize(7);

    for (int i=0;i<7;i++)
    {
        jointState1.position[i] = q_cmd[i];
        jointState2.position[i] = q_cmd[i+7];
    }

        /*
    jointState1.position[0] = q_cmd[0];
    jointState1.position[1] = q_cmd[1];
    jointState1.position[2] = q_cmd[2];
    jointState1.position[3] = q_cmd[3];
    jointState1.position[4] = q_cmd[4];
    jointState1.position[5] = q_cmd[5];
    jointState1.position[6] = q_cmd[6];*/
    cout << "jointState_PSM1" << jointState1 << endl;
    cout << "jointState_PSM2" << jointState2 << endl;  
    set_position_joint_publisher_PSM1.publish(jointState1);
    set_position_joint_publisher_PSM2.publish(jointState2);

}

void TrajActionServer::executeCB(const actionlib::SimpleActionServer<davinci_traj_streamer::trajAction>::GoalConstPtr& goal) {
    double traj_clock, dt_segment, dq_segment, delta_q_segment, traj_final_time;
    int isegment;
    trajectory_msgs::JointTrajectoryPoint trajectory_point0;

    Eigen::VectorXd qvec, qvec0, qvec_prev, qvec_new;
    // TEST TEST TEST
    //Eigen::VectorXd q_vec;
    //q_vec<<0.1,0.2,0.15,0.4,0.5,0.6,0.7;    

    ROS_INFO("in executeCB");

    g_count++; // keep track of total number of goals serviced since this server was started
    result_.return_val = g_count; // we'll use the member variable result_, defined in our class
    result_.traj_id = goal->traj_id;
    cout<<"received trajectory w/ "<<goal->trajectory.points.size()<<" points"<<endl;
    // copy trajectory to global var:
    new_trajectory = goal->trajectory; // 
    // insist that a traj have at least 2 pts
    int npts = new_trajectory.points.size();
    if (npts  < 2) {
        ROS_WARN("too few points; aborting goal");
        as_.setAborted(result_);
    } else { //OK...have a valid trajectory goal; execute it
        //got_new_goal = true;
        //got_new_trajectory = true;
        ROS_INFO("Cb received traj w/ npts = %d",npts);
        //cout << "Cb received traj w/ npts = " << new_trajectory.points.size() << endl;
        //trajectory_msgs::JointTrajectoryPoint trajectory_point0;
        //trajectory_point0 = new_trajectory.points[0];  
        //trajectory_point0 =  tj_msg.points[0];   
        //cout<<new_trajectory.points[0].positions.size()<<" =  new_trajectory.points[0].positions.size()"<<endl;
        //cout<<"size of positions[]: "<<trajectory_point0.positions.size()<<endl;
        cout << "subgoals: " << endl;
        int njnts; 
        for (int i = 0; i < npts; i++) {
            njnts = new_trajectory.points[i].positions.size();
            cout<<"njnts: "<<njnts<<endl;
            for (int j = 0; j < njnts; j++) { //copy from traj point to 7x1 vector
                cout << new_trajectory.points[i].positions[j] << ", ";
            }
            cout<<endl;
            cout<<"time from start: "<<new_trajectory.points[i].time_from_start.toSec()<<endl;
            cout << endl;
        }

        as_.isActive();

        working_on_trajectory = true;
        //got_new_trajectory=false;
        traj_clock = 0.0; // initialize clock for trajectory;
        isegment = 0;
        trajectory_point0 = new_trajectory.points[0];
        njnts = new_trajectory.points[0].positions.size();
        int njnts_new;
        qvec_prev.resize(njnts);
        qvec_new.resize(njnts);
        ROS_INFO("populating qvec_prev: ");
        for (int i = 0; i < njnts; i++) { //copy from traj point to Eigen type vector
            qvec_prev[i] = trajectory_point0.positions[i];
        }
        //cmd_pose_right(qvec0); //populate and send out first command  
        //qvec_prev = qvec0;
        cout << "start pt: " << qvec_prev.transpose() << endl;
    }
    myfile.open("example.txt");
    Eigen::VectorXd position_joint_error;
    std::pair<Eigen::VectorXd, Eigen::VectorXd> position_joint_current_;
    while (working_on_trajectory) {
        traj_clock += dt_traj;
        // update isegment and qvec according to traj_clock; 
        //if traj_clock>= final_time, use exact end coords and set "working_on_trajectory" to false 
        //ROS_INFO("traj_clock = %f; updating qvec_new",traj_clock);
        working_on_trajectory = update_trajectory(traj_clock, new_trajectory, qvec_prev, isegment, qvec_new);
        //cmd_pose_right(qvec_new); // use qvec to populate object and send it to robot
        //ROS_INFO("publishing qvec_new as command");
        //davinciJointPublisher.pubJointStatesAll(qvec_new);
        cout << "im here" << endl;
        command_joints(qvec_new);  //map these to all gazebo joints and publish as commands      
        qvec_prev = qvec_new;
        position_joint_error = get_joint_error(qvec_new);
        position_joint_current_ = get_joint_current();

        myfile << "Current time: " << traj_clock << endl;
        myfile << "Desired joint position_PSM1: [" << qvec_new[0] << ", " << qvec_new[1] << ", " << qvec_new[2] << ", " << qvec_new[3] << ", " << qvec_new[4] << ", " << qvec_new[5] << ", " << qvec_new[6] << "]"<< endl;
        myfile << "Actual joint position_PSM1: [" << position_joint_current_.first[0] << ", " << position_joint_current_.first[1] << ", " 
        << position_joint_current_.first[2] << ", " << position_joint_current_.first[3] << ", " << position_joint_current_.first[4] << ", " 
        << position_joint_current_.first[5] << ", " << position_joint_current_.first[6] << "]"<< endl;
        myfile << "Error joint position_PSM1: [" << position_joint_error[0] << ", " << position_joint_error[1] << ", " << position_joint_error[2] << ", " << position_joint_error[3] 
        << ", " << position_joint_error[4] << ", " << position_joint_error[5] << ", " << position_joint_error[6] << "]"<< endl;


        myfile << "Desired joint position_PSM2: [" << qvec_new[7] << ", " << qvec_new[8] << ", " << qvec_new[9] << ", " << qvec_new[10] << ", " << qvec_new[11] << ", " << qvec_new[12] << ", " << qvec_new[13] << "]"<< endl;
        myfile << "Actual joint position_PSM2: [" << position_joint_current_.second[0] << ", " << position_joint_current_.second[1] << ", " 
        << position_joint_current_.second[2] << ", " << position_joint_current_.second[3] << ", " << position_joint_current_.second[4] << ", " 
        << position_joint_current_.second[5] << ", " << position_joint_current_.second[6] << "]"<< endl;
        myfile << "Error joint position_PSM2: [" << position_joint_error[7] << ", " << position_joint_error[8] << ", " << position_joint_error[9] << ", " << position_joint_error[10] 
        << ", " << position_joint_error[11] << ", " << position_joint_error[12] << ", " << position_joint_error[13] << "]"<< endl;
        //myfile << "Actual joint position: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]" << endl; 
        //3myfile << "Error joint position: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]" << endl;    
        //cout << "traj_clock: " << traj_clock << "; vec:" << qvec_new.transpose() << endl;
        ros::spinOnce();
        ros::Duration(dt_traj).sleep();
    }
    myfile.close();
    ROS_INFO("completed execution of a trajectory" );
    as_.setSucceeded(result_); // tell the client that we were successful acting on the request, and return the "result" message 
}

// more general version--arbitrary number of joints
bool TrajActionServer::update_trajectory(double traj_clock, trajectory_msgs::JointTrajectory trajectory, Eigen::VectorXd qvec_prev, 
        int &isegment, Eigen::VectorXd &qvec_new) {
    
    trajectory_msgs::JointTrajectoryPoint trajectory_point_from, trajectory_point_to;
    int njnts = qvec_prev.size();
    cout<<"njnts for qvec_prev: "<<njnts<<endl;
    Eigen::VectorXd qvec, qvec_to, delta_qvec, dqvec;
    int nsegs = trajectory.points.size() - 1;
    ROS_INFO("update_trajectory: nsegs = %d, isegment = %d",nsegs,isegment);
    double t_subgoal;
    //cout<<"traj_clock = "<<traj_clock<<endl;
    if (isegment < nsegs) {
        trajectory_point_to = trajectory.points[isegment + 1];
        t_subgoal = trajectory_point_to.time_from_start.toSec();
        cout<<"iseg = "<<isegment<<"; t_subgoal = "<<t_subgoal<<endl;
    } else {
        cout << "reached end of last segment" << endl;
        trajectory_point_to = trajectory.points[nsegs];
        t_subgoal = trajectory_point_to.time_from_start.toSec();
        
        for (int i = 0; i < njnts; i++) {
            qvec_new[i] = trajectory_point_to.positions[i];
        }
        cout << "final time: " << t_subgoal << endl;
        return false;
    }

    cout<<"iseg = "<<isegment<<"; t_subgoal = "<<t_subgoal<<endl;
    while ((t_subgoal < traj_clock)&&(isegment < nsegs)) {
        cout<<"loop: iseg = "<<isegment<<"; t_subgoal = "<<t_subgoal<<endl;
        isegment++;
        if (isegment > nsegs - 1) {
            //last point
            trajectory_point_to = trajectory.points[nsegs];
            cout<<"next traj pt #jnts = "<<trajectory_point_to.positions.size()<<endl;
            for (int i = 0; i < njnts; i++) {
                qvec_new[i] = trajectory_point_to.positions[i];
            }
            cout << "iseg>nsegs" << endl;
            return false;
        }

        trajectory_point_to = trajectory.points[isegment + 1];
        t_subgoal = trajectory_point_to.time_from_start.toSec();
    }
    //cout<<"t_subgoal = "<<t_subgoal<<endl;
    //here if have a valid segment:
    cout<<"njnts of trajectory_point_to: "<<trajectory_point_to.positions.size()<<endl;
    qvec_to.resize(njnts);
    for (int i = 0; i < njnts; i++) {
        qvec_to[i] = trajectory_point_to.positions[i];
    }
    delta_qvec.resize(njnts);
    delta_qvec = qvec_to - qvec_prev; //this far to go until next node;
    double delta_time = t_subgoal - traj_clock;
    if (delta_time < dt_traj) delta_time = dt_traj;
    dqvec.resize(njnts);
    dqvec = delta_qvec * dt_traj / delta_time;
    qvec_new = qvec_prev + dqvec;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_interpolator_action_server"); // name this node     
    ros::NodeHandle nh;
    //DavinciJointPublisher davinciJointPublisher(&nh);//(nh_); //DavinciJointPublisher davinciJointPublisher(&nh);

    //publisher is global
    //joint_cmd_pub_right = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 1);

    /* maybe restore this later...
    ROS_INFO("Initializing Services");
    ros::ServiceServer statusService = nh.advertiseService("trajInterpStatusSvc", trajInterpStatusSvc);
    */
   
    ROS_INFO("instantiating the trajectory interpolator action server: ");
    TrajActionServer as(nh); //(nh); // create an instance of the class "TrajActionServer"  
    //ros::Time begin = ros::Time::now();

    //thread thread_1(boost::bind(&TrajActionServer::home, this));
    //thread thread_2(boost::bind(&TrajActionServer::check_robot_state, this));
    
    //thread_1.join();
    //thread_2.join();
    ros::Duration(3).sleep();
    as.get_robot_state();
    cout << "start homing" << endl;
    as.home();
    /*
    thread thread_1(&TrajActionServer::home, &as);
    thread thread_2(&TrajActionServer::check_robot_state, &as);
    
    thread_2.join();
    thread_1.join();
    */
    //as.home();
    ROS_INFO("ready to receive/execute trajectories");
    //ros::Time end = ros::Time::now();
    //cout << "time difference " << end-begin << endl;
    //main loop:
    as.get_robot_state();
    //ros::spin();
    while (ros::ok()) {
        ros::spinOnce();
        ros::Duration(dt_traj).sleep();
    }
}
