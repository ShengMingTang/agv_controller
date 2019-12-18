#include <ros/ros.h>
#include <string>
#include "Controller.h"
using namespace std;
int main(int _argc, char** _argv)
{
    string nodename = search_nodename(_argc, _argv);
    ros::init(_argc, _argv, (string("control_cpp") + nodename).c_str());
    tircgo::Controller a(nodename);
    a.setup(_argc, _argv);
    while(ros::ok() && a.ok()){
        a.loopOnce();
        ros::spinOnce();
    }
    ROS_INFO("Process closed successfully");
    return 0;
}