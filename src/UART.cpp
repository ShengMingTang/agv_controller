#include "UART.h"

#ifdef ROBOT_CONTROLLER_TEST
    static int16_t node_ct = 1;
#endif

UART::UART(const string& _parent_frame_id)
:frame_id {_parent_frame_id + "/uart"}
,invoke_clt {this->n.serviceClient<RobotInvoke>(ROBOTINVOKE_TOPIC)}
,lastdriveTime {ros::Time::now()}
{
    ROS_INFO("[UART] constructed");
}
UART::~UART()
{
    ROS_INFO("[UART] destructed");
}
RobotInvoke UART::invoke(const int16_t _op, std::vector<int16_t> _args)
{
    RobotInvoke srv;
    srv.request.header.stamp = ros::Time::now();
    srv.request.header.frame_id = this->frame_id;
    srv.request.operation = _op;
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
    #endif
    bool success = this->invoke_clt.call(srv);
    #ifdef ROBOT_CONTROLLER_TEST
        success = true;
    #endif
    if(!success){
        ROS_ERROR("|---X---> UART(Srv) failed");
    }
    else{
        tircgo_controller::scheduleActionGoal::_goal_type goal;
        double t = (ros::Time::now() - this->lastdriveTime).toSec();
        switch (_op)
        {
            case OPCODE_DRIVE:
                _args.push_back(static_cast<int16_t>(t * 1000));
                if(this->cmds.empty()){
                    t = 0;
                }
                else if(this->cmds.back().act == OPCODE_DRIVE){
                    auto lastmov = this->cmds.back();
                    if(lastmov.args[0] == _args[0] && lastmov.args[1] == _args[1]){ // same motion
                        _args[2] += lastmov.args[2];
                        if(_args[0] == 0 && _args[1] == 0){
                            _args[2] = 0;
                        }
                        this->cmds.pop_back();
                    }
                    // may add more motion cancelation
                }
                goal.act = _op;
                goal.args = _args;
                this->cmds.push_back(goal);
                this->lastdriveTime = ros::Time::now();
                break;
            case OPCODE_CALIB:
            case OPCODE_TRAIN_BEGIN:
            case OPCODE_SETNODE:
            case OPCODE_TRAIN_FINISH:
                goal.act = _op;
                goal.args = _args;
                this->cmds.push_back(goal);
                break;
            default:
                break;
        }
    }
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
                ROS_INFO("[UART] Work finished");
                break;
            case OPCODE_CALIB:
               ROS_INFO("[UART] End Calib");
                break;
            case OPCODE_TRAIN_BEGIN:
                ROS_INFO("[UART] Begin Training @ R%d", _srv.request.argument[0]);
                break;
            case OPCODE_TRAIN_FINISH:
                ROS_INFO("[UART] Finish training");
                break;
            case OPCODE_SETNODE:
                break;
            default:
                break;
        }
    }
    return ret;
}
list<tircgo_controller::scheduleActionGoal::_goal_type> UART::get_cmds()
{
    stringstream ss;
    for(auto it : this->cmds){
        ss << (char)it.act << " ";
        for(auto it2 : it.args){
            ss << it2 << " ";
        }
        if(it.act == OPCODE_TRAIN_FINISH){
            ss << "\n --------------------- ";
        }
        ss << "\n";
    }
    ROS_INFO("cmds : %s", ss.str().c_str());
    return this->cmds;
}
void UART::clear()
{
    this->cmds.clear();
    this->lastdriveTime = ros::Time::now();
    ROS_INFO("[UART] clear");
}