#include "UART.h"

#ifdef ROBOT_CONTROLLER_TEST
    static int16_t node_ct = 1;
#endif

UART::UART(const string& _parent_frame_id)
:frame_id {_parent_frame_id + "/uart"}
,invoke_clt {this->n.serviceClient<RobotInvoke>(ROBOTINVOKE_TOPIC)}
{
    ROS_INFO("UART constructed");
}
UART::~UART()
{
    ROS_INFO("UART destructed");
}
RobotInvoke UART::invoke(const int16_t _op, const std::vector<int16_t> _args)
{
    RobotInvoke srv;
    srv.request.header.stamp = ros::Time::now();
    srv.request.header.frame_id = this->frame_id;
    srv.request.operation = (char)_op;
    srv.request.argument = _args;
    #ifdef ROBOT_CONTROLLER_TEST
        srv.response.is_legal_op = srv.response.is_arg_valid = srv.response.is_activated = true;
        srv.response.error_code = ERRCODE_OK;
        switch (_op)
        {
        case OPCODE_SIGNAL:
            // ROS_INFO("[UART] signal recerived");
            break;
        case OPCODE_SETNODE:
            srv.response.feedback = vector<int16_t>(1, node_ct++);
            break;
        case OPCODE_TRAIN_BEGIN:
            node_ct = 1;
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
    bool ret = _srv.response.is_legal_op && _srv.response.is_arg_valid && _srv.response.is_activated && _srv.response.error_code == ERRCODE_OK;
    #ifdef ROBOT_CONTROLLER_TEST
        ret = true;
    #endif
    if(!ret){
        stringstream ss;
        ss << "Op = " << _srv.request.operation << ", ";
        if(!_srv.response.is_legal_op){
            ss << "Op ill. , ";
        }
        if(!_srv.response.is_arg_valid){
            ss << "Args ill. , ";
        }
        else if(!_srv.response.is_activated){
            ss << "UART---X---> AGV fail, ";
        }
        ss << "Err: " << _srv.response.error_code;
            ROS_ERROR("%s", ss.str().c_str());
    }
    else{
        switch (_srv.request.operation)
        {
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
                ROS_INFO("[UART] Head for R%dN%d", _srv.request.argument[0], _srv.request.argument[1]);
                break;
            case OPCODE_WORK_FINISH:
                ROS_INFO("[UART] Work finished\n");
                break;
            case OPCODE_CALIB:
               ROS_INFO("[UART] End Calib\n");
                break;
            case OPCODE_TRAIN_BEGIN:
                ROS_INFO("[UART] Begin Training @ R%d", _srv.request.argument[0]);
                break;
            case OPCODE_TRAIN_FINISH:
                ROS_INFO("[UART] Finish training\n");
                break;
            case OPCODE_SETNODE:
                break;
            default:
                break;
        }
    }
    return ret;
}