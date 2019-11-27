#ifndef SCHEDULER_H
#define SCHEDULER_H

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <tircgo_controller/scheduleAction.h> // Note: "Action" is appended
#include "Control_proto.h"

typedef actionlib::SimpleActionClient<tircgo_controller::scheduleAction> Client;
typedef actionlib::SimpleActionServer<tircgo_controller::scheduleAction> Server;

int argc_;
char **argv_;
struct base
{
    base() {ros::init(argc_, argv_, "scheduler");}
};
struct empty : public base
{
    ros::NodeHandle nh_;
};

/* static vars */
static empty empty_;
static actionlib::SimpleActionClient<tircgo_controller::scheduleAction> ac_(ROBOT_SCHEDULER_CONTROLLER, true);
static tircgo_controller::scheduleGoal goal_;
static bool finished_before_timeout_;

using namespace tircgo;
using namespace tircgo_controller;

/* macro functions */
#define Wait \
{\
    ros::Rate r(1);\
    do{\
        ac_.waitForServer();\
        ac_.sendGoal(goal_);\
        ac_.waitForResult();\
        r.sleep();\
    }while(ros::ok() && ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED);\
}
#define Go(r, n) \
{\
    goal_.act = "go", goal_.args = {r, n};\
    ROS_INFO("Go to R%dN%d", r, n);\
    Wait;\
}
#define Delay(n) \
{\
    goal_.act = "delay", goal_.args = {n};\
    ROS_INFO("Delay for %f sec(s)", (double)n / 1000);\
    Wait;\
}
#define Drive(linear, angular) \
{\
    goal_.act = "drive", goal_.args = {linear, angular};\
    ROS_INFO("Drive at (%f, %f)", linear, angular);\
    Wait;\
}

/* code structure */
void setup();
void loop();

int main(int argc, char** argv)
{
    ROS_INFO("Wait for Controller ...");
    ac_.waitForServer();
    ROS_INFO("Controller connected");
    ROS_INFO("Scheduler will start after the start button on the joystick is pressed");
    setup();
    while(ros::ok()){
        loop();
    }
}

#endif