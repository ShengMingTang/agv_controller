#include "Wifi.h"

Wifi::Wifi(const string& _parent_frame_id):
frame_id{_parent_frame_id + "/wifi"}
,send_clt{this->n.serviceClient<wifi::WifiSend>(ROBOT_WIFI_SEND_SRV)}
,recv_sub{this->n.subscribe(ROBOT_WIFI_TOPIC, MSG_QUE_SIZE, &Wifi::recv_callback, this)}
{
    ROS_INFO("Wifi constructed");
}

Wifi::~Wifi()
{
    ROS_INFO("Wifi destructed");
}

void Wifi::recv_callback(wifi::WifiIO::ConstPtr _msg)
{
    this->que.push(_msg);
}
bool Wifi::is_msg_valid(wifi::WifiIO::ConstPtr _msg)
{
    //check
    if(_msg){
        return _msg->error_code == WIFI_ERR_NONE;
    }
    else
        return false;
}
wifi::WifiIO::ConstPtr Wifi::pop()
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