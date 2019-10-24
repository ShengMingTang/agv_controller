#include "Wifi.h"

Wifi::Wifi(const string& _parent_frame_id):
frame_id{_parent_frame_id + "/wifi"}
,send_clt{this->n.serviceClient<tircgo_msgs::WifiSend>(ROBOT_WIFI_SEND_SRV)}
,nodeocp_clt{this->n.serviceClient<tircgo_msgs::WifiNodeOcp>(ROBOT_WIFI_NODEOCP_OUTER)}
,recv_sub{this->n.subscribe(ROBOT_WIFI_TOPIC, MSG_QUE_SIZE, &Wifi::recv_callback, this)}
{
    ROS_INFO("Wifi constructed");
}

Wifi::~Wifi()
{
    ROS_INFO("Wifi destructed");
}
/* send on behalf of controller */
void Wifi::send(const string& _hint)
{

}
/* true then can enter, else must wait */
bool Wifi::node_ocp(const RouteNode& _nd)
{
    // request a service to Ng
    tircgo_msgs::WifiNodeOcp srv;
    if(!this->nodeocp_clt.call(srv)){
        ROS_ERROR("<NodeOcp Srv-Err>");
        return false;
    }
    return srv.response.is_ocp;
}
bool Wifi::is_msg_valid(WifiIO::ConstPtr _msg)
{
    // check
    if(_msg){
        return _msg->error_code == (char)WIFI_ERR_NONE;
    }
    else
        return false;
}
WifiIO::ConstPtr Wifi::pop()
{
    while(!this->que.empty()){
        auto _msg = this->que.front();
        if(this->is_msg_valid(_msg)){
            this->que.pop();
            return _msg;
        }
        else{
            this->que.pop();
            ROS_WARN("Discard a wifi msg");
        }
    }
    return nullptr;
}
void Wifi::recv_callback(WifiIO::ConstPtr _msg)
{
    while(this->que.size() >= WIFI_BUFF_SIZE)
        this->que.pop();
    this->que.push(_msg);
}