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
        if(!this->invoke_clt.call(srv)){ // success
            ROS_ERROR("|---X---> UART(Srv) failed");
        }
    #endif
    return srv;
}
bool UART::is_invoke_valid(const RobotInvoke  &_srv)
{
    bool ret = _srv.response.is_legal_op && _srv.response.is_arg_valid && _srv.response.is_activated;
    if(!ret){
        stringstream ss;
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
            ROS_ERROR(ss.str().c_str());
    }
    else{
        switch (_srv.request.operation)
        {
            // normal
            case OPCODE_RT_UP:
            case OPCODE_RT_DOWN:
            case OPCODE_ND_UP:
            case OPCODE_ND_DOWN:
            case OPCODE_DRIVE:
                this->invoke(OPCODE_SIGNAL, {DEVICE_BEEPER, DEVICE_BEEPER_2S, 1});
                break;
            // priviledged
            case OPCODE_SHUTDOWN:
            case OPCODE_CALIB:
            case OPCODE_TRAIN_BEGIN:
            case OPCODE_SETNODE:
            case OPCODE_TRAIN_FINISH:
            case OPCODE_WORK_BEGIN:
            case OPCODE_WORK_FINISH:
                this->invoke(OPCODE_SIGNAL, {DEVICE_BEEPER, DEVICE_BEEPER_L, 1});
                break;
            default:
                break;
        }
        switch (_srv.request.operation)
        {
            case OPCODE_WORK_BEGIN:
                ROS_INFO("Head for R%dN%d", _srv.request.argument[0], _srv.request.argument[1]);
                break;
            case OPCODE_WORK_FINISH:
                ROS_INFO("Work finished");
            case OPCODE_CALIB:
               ROS_INFO("End Calib");
               break; 
            case OPCODE_TRAIN_BEGIN:
                ROS_INFO("Begin Training @ R%d", _srv.request.argument[0]);
                break;
            case OPCODE_TRAIN_FINISH:
                ROS_INFO("Finish training");
                break;
            case OPCODE_SETNODE:
                ROS_INFO("Node #%d set successfully", _srv.response.feedback[0]);
                break;
            default:
                break;
        }
    }
    return ret;
}