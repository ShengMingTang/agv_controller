#ifndef WIFI_H
#define WIFI_H
#include <ros/ros.h>
#include <queue>
#include "Control_proto.h"
// #include "Controller.h"
#include "tircgo_msgs/Ask_Data.h"
#include "tircgo_msgs/Send_Task.h"
#include "tircgo_msgs/WifiNodeCost.h"
#include "tircgo_msgs/WifiNodeOcp.h"
#include "tircgo_msgs/WifiTaskConfirm.h"
using namespace std;
using namespace pybot;
using namespace tircgo_msgs;
using Node = tircgo_msgs::RouteNode;
namespace pybot
{
    class Wifi
    {
    public:
        Wifi(const string& _parent_frame_id, Controller *const _parent);
        ~Wifi();
        // void send(const string& _hint);
        bool is_node_ocp(const RouteNode& _nd);
        // bool is_msg_valid(WifiIO::ConstPtr _msg);
        // WifiIO::ConstPtr pop();
        // bool empty()const {return this->que.empty();}
    private:
        ros::NodeHandle n;
        const string frame_id;
        Controller *const parent;

        ros::ServiceServer nodeocp_srv;
        ros::ServiceClient nodeocp_clt;
        bool nodeocp_serve(WifiNodeOcp::Request &_req, WifiNodeOcp::Response &_res);

        ros::ServiceServer nodecost_srv;
        ros::ServiceClient nodecost_clt;
        bool nodecost_serve(WifiNodeCost::Request &_req, WifiNodeCost::Response &_res);

        // queue<WifiIO::ConstPtr> que;
        // ros::ServiceClient send_clt; //send wifi
        // ros::Subscriber recv_sub; // receiver
        // void recv_callback(WifiIO::ConstPtr _msg);
    };
}
#endif