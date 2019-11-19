#include <ros/ros.h>
#include <string>
#include "Controller.h"
using namespace std;
int main(int _argc, char** _argv)
{
    ros::init(_argc, _argv, "controller_test");
    ros::NodeHandle nh;
    tircgo::Controller a("00");
    a.setup(_argc, _argv);
    while(ros::ok() && a.ok()){
        a.loopOnce();
        ros::spinOnce();
    }
    return 0;
}