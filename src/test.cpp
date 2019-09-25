#include <ros/ros.h>
#include "Controller.h"

int main(int _argc, char** _argv)
{
    ros::init(_argc, _argv, "controller_test");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    pybot::Controller a("/01");
    a.setup();
    while(1){
        a.loopOnce();
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}