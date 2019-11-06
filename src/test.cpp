#include <ros/ros.h>
#include <string>
#include "Controller.h"
using namespace std;
int main(int _argc, char** _argv)
{
    ros::init(_argc, _argv, "controller_test");
    ros::NodeHandle nh;
    ros::Rate loop_rate(ROBOT_LOOP_FREQ);
    tircgo::Controller a("00");
    a.setup();
    while(ros::ok()){
        if(a.ok()){
            a.loopOnce();
            a.monitor_display();
            loop_rate.sleep();
            ros::spinOnce();
        }
        else{
            break;
        }
    }
    return 0;
}