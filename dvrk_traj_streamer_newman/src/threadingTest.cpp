#include <chrono>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

bool is_ready(false);
std::mutex m;
std::condition_variable cv;

void
test()
{
    std::unique_lock<std::mutex> lk(m);
    get_robot_state();
    is_ready = true;
    cv.notify_one();
}

int
main()
{
    std::thread t(test);
    std::unique_lock<std::mutex> lk(m);
    while (!is_ready)
    {
        cv.wait_for(lk, 20);
        if (!is_ready)
            std::cout << "Spurious wake up!\n";
    }
    t.join();
}

void waits()
{
    std::unique_lock<std::mutex> lk(cv_m);
    std::cout << "Waiting... \n";
    cv.wait(lk, []{return i == 1;});
    std::cout << "...finished waiting. i == 1\n";
    done = true;
}
 


void TrajActionServer::home()
{
/*
        """This method will provide power to the robot as will as home
        the robot. This method requries the robot name."""*/
        //rospy.loginfo(rospy.get_caller_id() + ' -> start homing')
        std_msgs::String state;
        state.data = "Home";
        if(robot_state.compare(state.data)==0)
        {
            return true;
        } 
        dvrk_set_state(state);
        int counter = 10; // up to 10 transitions to get ready
        cout << "robot_state: " << get_robot_state() << endl;
        while (counter > 0){
            cout << "robot_state: " << get_robot_state() << endl;
            cv.wait_for(lk, 20); // give up to 20 secs for each transition
            if (robot_state.compare("DVRK_READY") != 0){
                counter = counter - 1;
                ROS_INFO("waiting for state to be DVRK_READY");
            }
                //rospy.loginfo(rospy.get_caller_id() + ' -> waiting for state to be DVRK_READY')
            else{
                counter = -1;
            }
        }
        if (robot_state.compare("DVRK_READY") != 0)
        {
            ROS_INFO("failed to reach state DVRK_READY");
            //rospy.logfatal(rospy.get_caller_id() + ' -> failed to reach state DVRK_READY')
        }
        
        ROS_INFO("homing complete");
        //rospy.loginfo(rospy.get_caller_id() + ' <- homing complete')

}