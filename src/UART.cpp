#include "UART.h"

#if ROBOT_CONTROLLER_TEST
    static int16_t node_ct;
#endif

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
RobotInvoke UART::invoke(const int16_t _op, const std::vector<int16_t> _args)
{
    RobotInvoke srv;
    srv.request.header.stamp = ros::Time::now();
    srv.request.header.frame_id = this->frame_id;
    srv.request.operation = (char)_op;
    srv.request.argument = _args;
    #if ROBOT_CONTROLLER_TEST
        srv.response.is_legal_op = srv.response.is_arg_valid = srv.response.is_activated = true;
        srv.response.error_code = ERRCODE_OK;
        switch (_op)
        {
        case OPCODE_SIGNAL:
            ROS_INFO("[UART] signal recerived");
            break;
        case OPCODE_SETNODE:
            srv.response.feedback = vector<int16_t>(1, node_ct++);
            break;
        case OPCODE_TRAIN_BEGIN:
            node_ct = 0;
            break;
        default:
            break;
        }
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
    else if(!_srv.response.is_activated){
        ss << "UART---X---> AGV fail, ";
        ret = false;
    }
    ss << "Err: " << _srv.response.error_code;
    if(!ret){
        switch (_srv.request.operation)
        {
            case OPCODE_SHUTDOWN:
                ROS_ERROR("<Shutdownroff Srv-Err>");
                break;
            case OPCODE_DRIVE:
                ROS_ERROR("<Drive Srv-Err>");
                break;
            case OPCODE_CALIB:
                ROS_ERROR("<Calib Srv-Err>");
                break;
            case OPCODE_TRAIN_BEGIN:
                ROS_ERROR("<Traing begin Srv-Err>");
                break;
            case OPCODE_SETNODE:
                ROS_ERROR("<SetNode Srv-Err>");
                break;
            case OPCODE_TRAIN_FINISH:
                ROS_ERROR("<Traing finished Srv-Err>");
                break;
            case OPCODE_WORK_BEGIN:
                ROS_ERROR("<Working begin Srv-Err>");
                break;
            case OPCODE_WORK_FINISH:
                ROS_ERROR("<Working finish Srv-Err>");
                break;     
            default:
                break;
        }
        ROS_ERROR(ss.str().c_str());
    }
    return ret;
}