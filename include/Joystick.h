#ifndef JOYSTICK_H
#define JOYSTICK_H
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <queue>
#include <string>
#include <functional>
#include "Control_proto.h"

#define JOY_BUFF_SIZE 50
using namespace std;
using namespace pybot;

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
        void callback(const sensor_msgs::Joy::ConstPtr& _msg);
        
        /* runtime */
        queue<sensor_msgs::Joy::ConstPtr> que;
        ros::Time last_press_t;
    };
}

#endif