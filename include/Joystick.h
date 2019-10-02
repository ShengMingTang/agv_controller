#ifndef JOYSTICK_H
#define JOYSTICK_H
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <queue>
#include <string>
#include <functional>
#include "pybot.h"

#define JOY_VERBOSE 0

using namespace std;
using namespace pybot;
// using namespace tircgo_uart;
namespace pybot
{
    class Joystick
    {
    public:
        Joystick(const string& _parent_frame_id);
        ~Joystick();
        sensor_msgs::Joy::ConstPtr pop();
    private:
        ros::NodeHandle n;
        const string frame_id;
        ros::Subscriber sub;
        
        sensor_msgs::Joy::ConstPtr ptr;
        void callback(const sensor_msgs::Joy::ConstPtr& _msg);
        /* runtime */
        bool is_frozen = false;
        // ros::Timer timer;
        void timer_callback(const ros::TimerEvent& _event);
        ros::Time last_press_t;
    };
}

#endif