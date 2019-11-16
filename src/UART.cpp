#include "UART.h"
UART::UART(const string& _parent_frame_id)
:frame_id {_parent_frame_id + "/uart"}
,invoke_clt {this->n.serviceClient<RobotInvoke>(ROBOTINVOKE_TOPIC)}
{
    ROS_INFO("UART constructed");
}
UART::~UART()
{
    ROS_INFO("UART destroyed");
}
RobotInvoke UART::invoke(const Opcode _op, const std::vector<int16_t> _args)
{
    RobotInvoke srv;
    srv.request.header.stamp = ros::Time::now();
    srv.request.header.frame_id = this->frame_id;
    srv.request.operation = (char)_op;
    srv.request.argument = _args;
    #if ROBOT_CONTROLLER_TEST
        srv.response.is_legal_op = srv.response.is_arg_valid = srv.response.is_activated = true;
        srv.response.error_code = 1;
    #else
        if(!this->invoke_clt.call(srv)){
            ROS_ERROR("|---X---> UART(Srv) failed");
        }
    #endif
    return srv;
}
bool UART::is_invoke_valid(RobotInvoke _srv)
{
    stringstream ss;
    bool ret = true;
    ss << "Op = " << _srv.request.operation << ", ";
    if(!_srv.response.is_legal_op){
        ss << "Op ill. , ";
        ret = false;
    }
    if(!_srv.response.is_arg_valid){
        ss << "Args ill. , ";
        ret = false;
    }
    else

    if(!_srv.response.is_activated){
        ss << "UART---X---> AGV fail, ";
        ret = false;
    }
    ss << "Err: " << _srv.response.error_code;
    if(!ret){
        ROS_ERROR(ss.str().c_str());
    }
    return ret;
}