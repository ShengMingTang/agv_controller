#include "Controller.h"
Controller::Controller(const string& _id):
base_driver{UART(_id)}
,joystick{Joystick(_id)}
,tracking_status_sub{this->n.subscribe(ROBOTSTATUS_TOPIC, MSG_QUE_SIZE, &Controller::status_tracking, this)}
,vw{vector<int16_t>(2)}
#if CONTROLLER_VERBOSE
,monitor{this->n.advertise<std_msgs::String>(MONITOR_TOPIC, MSG_QUE_SIZE)}
#endif
{
    ROS_INFO("Controller constructed");
}
Opcode Controller::decode_opcode(sensor_msgs::Joy::ConstPtr& _ptr)
{
    Opcode ret = Opcode::OPCODE_NONE;
    if(_ptr){
        vector<float> axes = _ptr->axes;
        vector<int32_t> buttons = _ptr->buttons;
        // privileged instructions 
        if(buttons[JOYBUTTON_RT]){
            this->is_driving = false;
            ros::Duration t = ros::Time::now() - this->driving_starttime;
            this->driving_dist += abs(t.toSec() * this->vw[0]);
            this->vw = vector<int16_t>(2, 0);
            return Opcode::OPCODE_STOP;
        }
        if(buttons[JOYBUTTON_A])
            return Opcode::OPCODE_IDLE;
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
                break;
            case Mode::MODE_WORKING:
                if(buttons[JOYBUTTON_X])
                    ret = Opcode::OPCODE_WORK_BEGIN;
        }
        // decode vw
        if(ret == Opcode::OPCODE_NONE){
            if(!this->is_driving && (axes[JOYAXES_CROSS_UD] != 0 || axes[JOYAXES_CROSS_LR] != 0)){
                this->is_driving = true;
                this->driving_starttime = ros::Time::now();
                this->vw[0] = axes[JOYAXES_CROSS_UD] * MOTOR_LINEAR_LIMIT / 2;
                this->vw[1] = axes[JOYAXES_CROSS_LR] * MOTOR_ANGULAR_LIMIT / 2;
                if(this->vw[1]) this->vw[0] = 0;
            }
        }
    }
    return ret;
}
void Controller::decode_drive(sensor_msgs::Joy::ConstPtr& _ptr)
{
    if(_ptr){
        vector<int32_t> buttons = _ptr->buttons;
        this->vw[0] = _ptr->axes[0] * 100;
        this->vw[1] = _ptr->axes[1] * 100;
    }
}
void Controller::drive()
{
    if(this->is_driving && (this->vw[0] != 0 || this->vw[1] != 0)){
        this->base_driver.invoke((char)Opcode::OPCODE_DRIVE, this->vw);
        ROS_INFO("Drive with(%d, %d)", this->vw[0], this->vw[1]);
    }
}
void Controller::setup()
{
    ROS_INFO("Setup");
}
void Controller::isr()
{
    ROS_INFO("Jump into ISR, not implemented yet");
}
bool Controller::check_safety()
{
    ; // not implemented
}
void Controller::monitor_display()
{
    stringstream ss;
    ss << "mode: " << (int)this->mode;
    ss << ", (" << this->vw[0] << ", " << this->vw[1] << ")";
    ss << ", trk: " << (int)this->tracking_status << ", ";
    ss << "lidar:[";
    for(auto it : this->lidar_levels){
        ss << it << ",";
    }
    ss << "]";
    std_msgs::String msg;
    msg.data = ss.str();
    this->monitor.publish(msg);
}
void Controller::loopOnce()
{
    // safety issue not implemented yet
    
    auto op_ptr = this->get_joy_signal();
    this->op = this->decode_opcode(op_ptr);
    // this->decode_drive(op_ptr);

    // decode privileged instructions
    switch(this->op){
        case Opcode::OPCODE_POWEROFF:
            this->log();
            return;
        case Opcode::OPCODE_IDLE:
            ROS_WARN("Direct forced switch to Idle mode");
            #if CONTROLLER_TEST
                this->mode = Mode::MODE_IDLE;
            #endif
            return;
        default:
            break;
    }
    switch(this->mode){
        case Mode::MODE_IDLE:
            this->idle();
            break;
        case Mode::MODE_HOMING:
            this->homing();
            break;
        case Mode::MODE_TRAINING:
            if(this->is_calibed)
                this->training();
            else{
                ROS_WARN("Not Homed but want to enter Trainng mode");
                ROS_WARN("Direct forced switch to Idle mode");
                this->mode = Mode::MODE_IDLE;
            }
            break;
        case Mode::MODE_WORKING:
            if(this->is_trained)
                this->working();
            else{
                ROS_WARN("Not Trained but want to enter Working mode");
                ROS_WARN("Direct forced switch to Idle mode");
                this->mode = Mode::MODE_IDLE;
            }
            break;
        default:
            ROS_WARN("Undefined mode");
            break;
    }
    #if CONTROLLER_VERBOSE
        this->monitor_display();
    #endif
}
void Controller::log()
{
    ROS_INFO("Write log, not implemented yet");
}
void Controller::idle()
{
    RobotInvoke srv;
    switch(this->op){
        case Opcode::OPCODE_NONE:
            break;
        case Opcode::OPCODE_HOMEING:
            this->clear();
            srv = this->base_driver.invoke((char)Opcode::OPCODE_HOMEING, vector<int16_t>());
            #if CONTROLLER_TEST
                this->mode = Mode::MODE_HOMING;
            #endif
            break;
        case Opcode::OPCODE_TRAIN_BEGIN:
            srv = this->base_driver.invoke((char)Opcode::OPCODE_TRAIN_BEGIN, vector<int16_t>());
            #if CONTROLLER_TEST
                this->mode = Mode::MODE_TRAINING;
                node_ct = 0;
                this->training_route = route_ct++;
            #else
                if(this->base_driver.is_invoke_valid(srv)){
                    this->training_route = srv.response.feedback[0];
                }
            #endif
            break;
        case Opcode::OPCODE_WORK_BEGIN:
            this->working_route = this->working_node = 0;
            #if CONTROLLER_TEST
                this->mode = Mode::MODE_WORKING;
            #endif
            srv = this->base_driver.invoke((char)Opcode::OPCODE_WORK_BEGIN, vector<int16_t>());
            break;
        default:
            ROS_WARN("In Idle mode, %c is not allowed", (char)this->op);
            break;
    }
    this->drive();
}
void Controller::clear()
{
    ROS_INFO("Clear data, Not implemented yet");
    this->is_origin_set = false;
    this->is_calibed = false;
    this->is_calib_begin = false;
    this->is_trained = false;
    // graph clear
}
void Controller::homing()
{
    RobotInvoke srv;
    switch(this->op){
        case Opcode::OPCODE_NONE:
            break;
        case Opcode::OPCODE_ORIGIN:
            if(!this->is_origin_set){
                this->is_origin_set = true;
                srv = this->base_driver.invoke((char)Opcode::OPCODE_ORIGIN, vector<int16_t>());
                ROS_INFO("Origin set");
            }
            break;
        case Opcode::OPCODE_CALIB_BEGIN:
            if(this->is_origin_set){
                this->is_calib_begin = true;
                srv = this->base_driver.invoke((char)Opcode::OPCODE_CALIB_BEGIN, vector<int16_t>());
                ROS_INFO("Calibration begin");
            }
            break;
        case Opcode::OPCODE_CALIB_FINISH:
            if(this->is_calib_begin){
                this->is_calibed = true;
                this->is_calib_begin = false;
                srv = this->base_driver.invoke((char)Opcode::OPCODE_CALIB_FINISH, vector<int16_t>());
                ROS_INFO("Calibration finish, switch to Idle mode");
                #if CONTROLLER_TEST
                    this->mode = Mode(Mode::MODE_IDLE); // bug
                #endif
            }
            break;
        default:
            ROS_INFO("In Homing mode, %c is not allowed", (char)this->op);
            break;
    }
    this->drive();
}
void Controller::training()
{
    RobotInvoke srv;
    switch(this->op){
        case Opcode::OPCODE_NONE:
            break;
        case Opcode::OPCODE_SETNODE:
            srv = this->base_driver.invoke((char)Opcode::OPCODE_SETNODE, vector<int16_t>());
            #if CONTROLLER_TEST
                srv.response.feedback = vector<int16_t>(1, node_ct++);
                this->set_node(this->training_route, srv.response.feedback[0]);
            #else
                if(this->base_driver.is_invoke_valid(srv)){
                    this->set_node(this->training_route, srv.response.feedback[0]);
                }
                else{
                    ROS_ERROR("SetNode Invokation Ignored");
                }
            #endif
            break;
        case Opcode::OPCODE_TRAIN_FINISH:
            this->is_trained = true;
            srv = this->base_driver.invoke((char)Opcode::OPCODE_TRAIN_FINISH, vector<int16_t>());
            #if CONTROLLER_TEST
                this->mode = Mode::MODE_IDLE;
            #endif
            ROS_INFO("Training finished");
            break;
        default:
            ROS_INFO("In Training mode, %c is not allowed", (char)this->op);
            break;
    }
    this->drive();
}
void Controller::set_node(int16_t _route, int16_t _node)
{
    ROS_INFO("Set RouteNode on R%d, N%d, W%lf", _route, _node, this->driving_dist);
    this->driving_dist = 0;
}
void Controller::working()
{
    switch(this->op){
        case Opcode::OPCODE_NONE:
            break;
        case Opcode::OPCODE_TARGET_ROUTE_U:
            break;
        case Opcode::OPCODE_TARGET_ROUTE_D:
            break;
        case Opcode::OPCODE_TARGET_NODE_U:
            break;
        case Opcode::OPCODE_TARGET_NODE_D:
            break;
        default:
            ROS_WARN("In Working mode, %c is not allowed", (char)this->op);
            break;
    }
}
Controller::~Controller()
{
    ROS_INFO("Controller destructed");
}
sensor_msgs::Joy::ConstPtr Controller::get_joy_signal()
{
    return this->joystick.pop();
}
void Controller::status_tracking(const RobotStatus::ConstPtr& _msg)
{
    if(_msg->is_activated){
        ROS_INFO("RobotStatus Msg receiced");
        this->mode = (Mode)_msg->now_mode;
        if(_msg->jreply.is_activated){
            this->tracking_status = (Tracking_status)_msg->jreply.reply;
        }
        else{
            ROS_WARN("Jreply invalid");
        }
        if(_msg->lreply.is_activated){
            int dir = (int)_msg->lreply.dir_reply - 1;
            int16_t level = _msg->lreply.level_reply;
            (this->lidar_levels)[dir] = level;
        }
        else{
            ROS_WARN("Lreply invalid");
        }
    }
    else{
        ROS_WARN("RobotStatus Msg invalid");
    }
}