#include "UART.h"
UART::UART(const string& _parent_frame_id):
invoke_clt{this->n.serviceClient<RobotInvoke>(ROBOTINVOKE_TOPIC)}
// ,velCmd_pub{this->n.advertise<geometry_msgs::TwistStamped>(ROBOTVELCMD_TOPIC, MSG_QUE_SIZE)},
,frame_id{_parent_frame_id + "/uart"}
{
    ROS_INFO("UART constructed\n");
}
RobotInvoke UART::invoke(const char _op, const std::vector<int16_t>& _args)
{
    RobotInvoke srv;
    srv.request.header.stamp = ros::Time::now();
    srv.request.header.frame_id = this->frame_id;
    srv.request.operation = _op;
    srv.request.argument = _args;
    bool result = this->invoke_clt.call(srv);
    #if UART_VERBOSE
    ROS_INFO("Invoked");
    if(!result){
        ROS_ERROR("Failed to call invoke op %c\n", _op);
    }
    else{
        ROS_WARN("Connection to UART Service failed\n");
    }
    if(!this->is_invoke_valid(srv)){
        ;
    }
    #endif
    return srv;
}
UART::~UART()
{
    ROS_INFO("UART destroyed\n");
}
bool UART::is_invoke_valid(RobotInvoke _srv)
{
    bool ret = true;
    if(!_srv.response.is_legal_op){
        #if UART_VERBOSE
        ROS_WARN("Operation illegal\n");
        #endif
        ret = false;
    }
    if(!_srv.response.is_arg_valid){
        #if UART_VERBOSE
        ROS_WARN("Args Invalid\n");
        #endif
        ret = false;
    }
    if(!_srv.response.is_activated){
        #if UART_VERBOSE
        ROS_WARN("Connect to UART <-> AGV failed\n");
        #endif
        ret = false;
    }
    return ret;
}