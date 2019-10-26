#include "Wifi.h"

Wifi::Wifi(const string& _parent_frame_id, Controller *const _parent):
frame_id{_parent_frame_id + "/wifi"}
,parent{_parent}
,nodeocp_srv{this->n.advertiseService(ROBOT_WIFI_NODEOCP_INNER, &Wifi::nodeocp_serve, this)}
,nodeocp_clt{this->n.serviceClient<tircgo_msgs::WifiNodeOcp>(ROBOT_WIFI_NODEOCP_OUTER)}
,nodecost_srv{this->n.advertiseService(ROBOT_WIFI_NODECOST_INNER, &Wifi::nodecost_serve, this)}
,nodecost_clt{this->n.serviceClient<tircgo_msgs::WifiNodeOcp>(ROBOT_WIFI_NODECOST_OUTER)}
// ,send_clt{this->n.serviceClient<tircgo_msgs::WifiSend>(ROBOT_WIFI_SEND_SRV)}
// ,recv_sub{this->n.subscribe(ROBOT_WIFI_TOPIC, MSG_QUE_SIZE, &Wifi::recv_callback, this)}
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
bool Wifi::nodeocp_serve(WifiNodeOcp::Request &_req, WifiNodeOcp::Response &_res)
{
    Node nd = parent->get_nodecop();
    _res.is_ocp = (_req.q_rn.route == nd.route && _req.q_rn.node == nd.node);
    _res.error_code = WIFI_ERR_NONE;
    return true;
}
bool Wifi::nodecost_serve(WifiNodeCost::Request &_req, WifiNodeCost::Response &_res)
{
    _res.cost.target = _req.target;
    _res.cost = parent->get_cost_to_target(_req.target);
    _res.error_code = WIFI_ERR_NONE;
    return true;
}
/* true then can enter, else must wait */
bool Wifi::is_node_ocp(const RouteNode& _nd)
{
    // request a service from Ng
    tircgo_msgs::WifiNodeOcp srv;
    if(!this->nodeocp_clt.call(srv)){
        ROS_ERROR("<NodeOcp Srv-Err>");
        return false;
    }
    return srv.response.error_code == WIFI_ERR_NONE && srv.response.is_ocp;
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