#include "Controller.h"
Controller::Controller(const string& _id):
base_driver{UART(_id)}
,joystick{Joystick(_id)}
,tracking_status_sub{this->n.subscribe(ROBOTSTATUS_TOPIC, MSG_QUE_SIZE, &Controller::status_tracking, this)}
,monitor{this->n.advertise<std_msgs::String>(MONITOR_TOPIC, MSG_QUE_SIZE)}
,lidar_levels {vector<int16_t>(4, LIDAR_LEVEL_L)}
,rn_img{vector< vector<VertexType*> >(TRAIN_ROUTE_MAX, vector<VertexType*>() )}

,nodeocp_srv{this->n.advertiseService(ROBOT_WIFI_NODEOCP_INNER, &Controller::nodeocp_serve, this)}
,nodeocp_clt{this->n.serviceClient<tircgo_msgs::WifiNodeOcp>(ROBOT_WIFI_NODEOCP_OUTER)}
,nodecost_srv{this->n.advertiseService(ROBOT_WIFI_NODECOST_INNER, &Controller::nodecost_serve, this)}
,nodecost_clt{this->n.serviceClient<tircgo_msgs::WifiNodeOcp>(ROBOT_WIFI_NODECOST_OUTER)}

,task_confirm_srv{this->n.advertiseService(ROBOT_WIFI_TASK_CONFIRM_INNER, &Controller::task_confirm_serve, this)}

,askdata_srv{this->n.advertiseService(ROBOT_ASKDATA_TOPIC, &Controller::askdata_serve, this)}

{
    this->nd_training.route = this->nd_training.node = 0;
    this->ocp_vptr = nullptr, this->target_vptr = nullptr;
    ROS_INFO("Controller constructed");
}
Controller::~Controller()
{
    ROS_INFO("Controller destructed");
}
void Controller::setup()
{
    ROS_INFO("Controller Setup:");
    ROS_INFO("Loop at frequency %d", ROBOT_LOOP_FREQ);
    ROS_WARN("Only objects in front of Robot could block, neglect side objects");
    ROS_INFO("Setup Done");
    // can wait for UART in the future
}
void Controller::isr(const int _inter)
{
    // implement
    ROS_INFO("Call ISR but do nothing");
    // call for light twinking or others
}

/*
    Drive related
    decode opcode according to state
    Non-state-transition-related op will be decoded as OPCODE_NONE 
    and get executed in place
*/
Opcode Controller::decode_opcode()
{
    Opcode ret = Opcode::OPCODE_NONE;
    if(this->op_ptr){
        vector<float> axes = this->op_ptr->axes;
        vector<int32_t> buttons = this->op_ptr->buttons;
        if(buttons[JOYBUTTON_BACK])
            return Opcode::OPCODE_POWEROFF;
        
        // decode mode dependent instructions
        switch(this->mode){
            case Mode::MODE_IDLE:
                if(buttons[JOYBUTTON_Y])
                    ret = Opcode::OPCODE_HOMEING;
                else if(buttons[JOYBUTTON_X])
                    ret = Opcode::OPCODE_TRAIN_BEGIN;
                else if(buttons[JOYBUTTON_B])
                    ret = Opcode::OPCODE_WORK_BEGIN;
                else if(buttons[JOYBUTTON_RT])
                    this->nd_target.route++;
                else if(buttons[JOYBUTTON_LT])
                    if(this->nd_target.route - 1 >= 0)
                        this->nd_target.route--;
                else if(buttons[JOYBUTTON_RB])
                    this->nd_target.node++;
                else if(buttons[JOYBUTTON_LB])
                    if(this->nd_target.node - 1 >= 0)
                        this->nd_target.node--;
                break;
            case Mode::MODE_HOMING:
                if(buttons[JOYBUTTON_Y])
                    ret = Opcode::OPCODE_ORIGIN;
                else if(buttons[JOYBUTTON_X])
                    ret = Opcode::OPCODE_CALIB_BEGIN;
                else if(buttons[JOYBUTTON_B])
                    ret = Opcode::OPCODE_CALIB_FINISH;
                break;
            case Mode::MODE_TRAINING:
                if(buttons[JOYBUTTON_X])
                    ret = Opcode::OPCODE_SETNODE;
                else if(buttons[JOYBUTTON_B])
                    ret = Opcode::OPCODE_TRAIN_FINISH;
                else if(buttons[JOYBUTTON_RT] && this->nd_training.route < TRAIN_ROUTE_MAX)
                    this->nd_training.route++;
                else if(buttons[JOYBUTTON_LT] && this->nd_training.route > 0)
                    this->nd_training.route--;
                break;
            case Mode::MODE_WORKING:
                if(buttons[JOYBUTTON_B])
                    ret = Opcode::OPCODE_WORK_FINISH;
                break;
            default:
                break;
        }
    }
    return ret;
}
/*
    decode joystick to drive signal
    independent of Opcode decoding
*/
vector<int16_t> Controller::decode_drive()
{
    if(this->op_ptr){
        vector<float> axes = this->op_ptr->axes;
        vector<int32_t> buttons = this->op_ptr->buttons;
        // privileged instructions 
        if(buttons[JOYBUTTON_A]){
            return {0, 0};
        }
        // decode vw
        if(axes[JOYAXES_CROSS_UD] != 0 || axes[JOYAXES_CROSS_LR] != 0){
            int16_t vv = axes[JOYAXES_CROSS_UD] * MOTOR_LINEAR_LIMIT / 2;
            int16_t ww = axes[JOYAXES_CROSS_LR] * MOTOR_ANGULAR_LIMIT / 2;
            vv = (ww == 0) ? vv : 0;
            return {vv, ww};
        }
    }
    // empty drive cmd, kepp current vel
    return this->pose_tracer.get_vel();
}
/* 
    drive machine interface, locked if trained (not implemented)
    Call this only in Idle, Homing, Training
*/
void Controller::drive(vector<int16_t> _vel)
{
    // trained but not in training
    if((this->stage_bm & STAGE_TRAINED) && this->mode != Mode::MODE_TRAINING){
        ROS_INFO("Once Trained, lock motor if not in tranining mode !");
        return ;
    }
    // legal condition to drive
    double t = (ros::Time::now() - this->pose_tracer.get_starttime()).toSec();
    if(t >= ROBOT_CONTROLLER_DRIVE_TIMEOUT || _vel[0] != this->pose_tracer.get_vel()[0] || _vel[1] != this->pose_tracer.get_vel()[1]){ // different motion
        if(_vel[0] == 0 && this->pose_tracer.get_vel()[0] == 0 &&
           _vel[1] == 0 && this->pose_tracer.get_vel()[1] == 0){
               // don't send redundant 0 vel
        }
        else{
            // direct pass
            #if ROBOT_CONTROLLER_TEST
                this->pose_tracer.set_vw(_vel);
            #else
                bool is_safe = true;
                #if ROBOT_CONTROLLER_SAFE
                    is_safe = this->check_safety();
                #endif
                if(is_safe){
                    auto srv = this->base_driver.invoke((char)Opcode::OPCODE_DRIVE, {_vel[0], _vel[1]});
                    if(this->base_driver.is_invoke_valid(srv)){
                        this->pose_tracer.set_vw(_vel);
                    }
                    else{
                        ROS_ERROR("<Drive Srv-Err>");
                    }

                }
                else{
                    ROS_INFO("Unsafe condition, drive rejected");
                }
            #endif
        }
    }
}
/* Sys related */
void Controller::clear()
{
    this->stage_bm = 0;
    this->runtime_vars_mgr(RUNTIME_VARS_RESET);
    this->pose_tracer.clear();
    this->graph.clear();
    // reset all runtime RouteNodes
    #if ROBOT_CONTROLLER_TEST
        route_ct = node_ct = 0;
    #endif
    ROS_INFO("Clear data");
}
/* writing log files */
void Controller::log()
{
    ROS_INFO("Write log, should implment serialization");
}
/* poweroff routine */
bool Controller::poweroff()
{
    auto srv = this->base_driver.invoke((char)Opcode::OPCODE_POWEROFF, vector<int16_t>());
    if(this->base_driver.is_invoke_valid(srv)){
        this->log();
        this->stage_bm &= (~STAGE_OK);
        ROS_INFO("Power off!");
    }
    else{
        ROS_ERROR("<Poweroff Srv-Err>");
    }
}
/* safety issue */
bool Controller::check_safety()
{
    if(this->mode != Mode::MODE_WORKING){
        if(this->op_vel[0] > 0)
            return this->lidar_levels[LIDAR_DIR_FRONT] <= LIDAR_LEVEL_H;
        else if(this->op_vel[0] < 0)
            return this->lidar_levels[LIDAR_DIR_BACK] <= LIDAR_LEVEL_H;
        else
            return true;
    }
    else{
        // Working mode, special check
    }
}
/* subsriber to track status */
void Controller::status_tracking(const RobotStatus::ConstPtr& _msg)
{
    if(_msg->is_activated){
        #if !ROBOT_CONTROLLER_TEST
            this->mode = (Mode)_msg->now_mode;
            if(_msg->jreply.is_activated){
                this->tracking_status = (Tracking_status)_msg->jreply.reply;
            }
            else{
                ROS_WARN("Jreply invalid");
            }
            if(_msg->lreply.is_activated){
                this->lidar_levels = _msg->lreply.level_reply;
            }
            else{
                ROS_WARN("Lreply invalid");
            }
        #endif
    }
    else{
        ROS_WARN("RobotStatus not activated");
    }
}
/* interface for buffering joystick signal */
sensor_msgs::Joy::ConstPtr Controller::get_joy_signal()
{
    return this->joystick.pop();
}
/* echo status */
void Controller::monitor_display() const
{
    stringstream ss;
    char buff[100];
    auto coor = this->pose_tracer.get_coor();
    sprintf(buff, "mode:%d| trk:%d| p:(%+.1f,%+.1f %+.1f)| v:<%+2d,%+2d>| L:[%d,%d,%d,%d]| T<%d,%d>",
                (int)this->mode,
                (int)this->tracking_status,
                coor.x, coor.y, coor.w,
                this->pose_tracer.get_vel()[0], this->pose_tracer.get_vel()[1],
                this->lidar_levels[0], this->lidar_levels[1], this->lidar_levels[2], this->lidar_levels[2],
                this->nd_target.route, this->nd_target.node
                );
    std_msgs::String msg;
    msg.data = string(buff);
    this->monitor.publish(msg);
}
bool Controller::is_target_ocp(const VertexType *vptr)
{
    // request a service from Ng
    tircgo_msgs::WifiNodeOcp srv;
    srv.request.q_rn = *(vptr->aliases.begin());
    if(!this->nodeocp_clt.call(srv)){
        ROS_ERROR("<NodeOcp Srv-Err>");
        return false;
    }
    return !(srv.response.error_code == WIFI_ERR_NONE && srv.response.is_ocp);
}

void Controller::runtime_vars_mgr(bool _flag)
{
    if(_flag == RUNTIME_VARS_SET){ // get all input signals
        this->op_ptr = this->get_joy_signal();
        this->op = this->decode_opcode();
        this->op_vel = this->decode_drive();
    }
    else{
        this->op_vel = {0, 0};
        this->op_ptr = nullptr;
        this->op = Opcode::OPCODE_NONE;
    }
}
bool Controller::priviledged_instr()
{
    bool ret = true;
    switch(this->op){
        // return statements
        case Opcode::OPCODE_POWEROFF:
            this->poweroff();
            break;
        default:
            ret = false;
            break;
    }
    return ret;
}