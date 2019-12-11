#include <ros/ros.h>
#include <string>
#include "Controller.h"
using namespace std;
int main(int _argc, char** _argv)
{
    if(_argc > 1){
        ros::init(_argc, _argv, (string("control_cpp") + _argv[1]).c_str());
        ros::NodeHandle nh;
        tircgo::Controller a(_argv[1]);
        a.setup(_argc - 1, _argv + 1);
        while(ros::ok() && a.ok()){
            a.loopOnce();
            ros::spinOnce();
        }
    }
    else{
        ros::init(_argc, _argv, string("control_cpp").c_str());
        ros::NodeHandle nh;
        tircgo::Controller a("");
        a.setup(_argc, _argv);
        while(ros::ok() && a.ok()){
            a.loopOnce();
            ros::spinOnce();
        }
    }
    return 0;
}