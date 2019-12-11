#include <ros/ros.h>
#include <string>
#include "Controller.h"
using namespace std;
int main(int _argc, char** _argv)
{
    ros::init(_argc, _argv, _argv[1]);
    ros::NodeHandle nh;
    tircgo::Controller a(_argv[1]);
    a.setup(_argc - 1, _argv + 1);
    while(ros::ok() && a.ok()){
        a.loopOnce();
        ros::spinOnce();
    }
    return 0;
}