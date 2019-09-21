#include <ros/ros.h>
#include "Controller.h"

int main(int _argc, char** _argv)
{
    ros::init(_argc, _argv, "controller_test");
    pybot::Controller a("/01");
    return 0;
}