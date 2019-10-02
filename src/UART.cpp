#include "UART.h"
UART::UART(const string& _parent_frame_id):
invoke_clt{this->n.serviceClient<RobotInvoke>(ROBOTINVOKE_TOPIC)}
,frame_id{_parent_frame_id + "/uart"}
{
    ROS_INFO("UART constructed");
}
RobotInvoke UART::invoke(const char _op, const std::vector<int16_t> _args)
{
    RobotInvoke srv;
    srv.request.header.stamp = ros::Time::now();
    srv.request.header.frame_id = this->frame_id;
    srv.request.operation = _op;
    srv.request.argument = _args;
    bool is_success = this->invoke_clt.call(srv);
    if(!this->invoke_clt.call(srv))
        ROS_ERROR("Connection to UART Service failed");
    return srv;
}
UART::~UART()
{
    ROS_INFO("UART destroyed");
}
bool UART::is_invoke_valid(RobotInvoke _srv)
{
    stringstream ss;
    bool ret = true;
    ss << "Op = " << _srv.request.operation << " ";
    if(!_srv.response.is_legal_op){
        ss << "    : Operation illegal\n";
        ret = false;
    }
    if(!_srv.response.is_arg_valid){
        ss << "    : Args illegal\n";
        ret = false;
    }
    if(!_srv.response.is_activated){
        ss << "    : Connect to UART <-> AGV failed";
        ret = false;
    }
    if(!ret)
        ROS_ERROR(ss.str().c_str());
    return ret;
}