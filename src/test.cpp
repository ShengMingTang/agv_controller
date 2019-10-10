#include <ros/ros.h>
#include <string>
#include "Controller.h"
using namespace std;
int main(int _argc, char** _argv)
{
    ros::init(_argc, _argv, "controller_test");
    ros::NodeHandle nh;
    ros::Rate loop_rate(AGV_LOOP_FREQ);
    string id("123.456.789");
    if(_argc > 1)
        id = _argv[1];
    pybot::Controller a(id.c_str());
    a.setup();
    while(ros::ok()){
        if(a.ok()){
            a.loopOnce();
        }
        else{
            break;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}