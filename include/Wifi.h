#ifndef WIFI_H
#define WIFI_H
#include <ros/ros.h>
#include <queue>
#include "pybot.h"
#include "tircgo_msgs/WifiIO.h"
#include "tircgo_msgs/WifiSend.h"
using namespace std;
using namespace pybot;
using namespace tircgo_msgs;

namespace pybot
{
    class Wifi
    {
    public:
        Wifi(const string& _parent_frame_id);
        ~Wifi();
        void send(const string& _hint);
        bool node_ocp(const RouteNode& _nd);
        bool is_msg_valid(WifiIO::ConstPtr _msg);
        WifiIO::ConstPtr pop();
        bool empty()const {return this->que.empty();}
    private:
        void recv_callback(WifiIO::ConstPtr _msg);
        ros::NodeHandle n;
        const string frame_id;
        queue<WifiIO::ConstPtr> que;
        ros::ServiceClient send_clt; //send wifi
        ros::Subscriber recv_sub; // receiver
    };
}
#endif