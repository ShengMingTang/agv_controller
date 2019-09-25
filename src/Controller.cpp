#include "Controller.h"
Controller::Controller(const string& _id):
uart{UART(_id)}
,joystick{Joystick(_id)}
,tracking_status_sub{this->n.subscribe(ROBOTSTATUS_TOPIC, MSG_QUE_SIZE, &Controller::status_tracking, this)}
,vw{vector<int16_t>(2)}
{
    ROS_INFO("Controller constructed");
    this->debugger = (this->n).advertise<std_msgs::String>(DEBUG_TOPIC, MSG_QUE_SIZE);
}
Opcode Controller::decode_opcode(sensor_msgs::Joy::ConstPtr& _ptr)
{
    Opcode ret = Opcode::OPCODE_NONE;
    if(_ptr){
        vector<float> axes = _ptr->axes;
        vector<int32_t> buttons = _ptr->buttons;
        // privileged instructions 
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
                else if(axes[JOYBUTTON_CROSS_LR] > 0)
                    ret = Opcode::OPCODE_TARGET_ROUTE_U;
                else if(buttons[JOYBUTTON_CROSS_LR] < 0)
                    ret = Opcode::OPCODE_TARGET_ROUTE_D;
                else if(buttons[JOYBUTTON_CROSS_UD] > 0)
                    ret = Opcode::OPCODE_TARGET_NODE_U;
                else if(buttons[JOYBUTTON_CROSS_UD] < 0)
                    ret = Opcode::OPCODE_TARGET_NODE_D;
                break;
        }
    }
    return ret;
}
void Controller::decode_drive(sensor_msgs::Joy::ConstPtr& _ptr)
{
    if(_ptr){
        this->vw[0] = _ptr->axes[0] * 100;
        this->vw[1] = _ptr->axes[1] * 100;
    }
}
void Controller::drive()
{
    if(this->vw[0] != 0 || this->vw[1] != 0){
        uart.invoke((char)Opcode::OPCODE_DRIVE, this->vw);
        #if CONTORLLER_VERBOSE
        // stringstream ss;
        // std_msgs::String msg;
        // ss << "Drive with(" << this->vw[0] << this->vw[1] << ")";
        // msg.data = ss.str();
        // (this->debugger).publish(msg);
        #endif
        ROS_INFO("Drive with(%d, %d)", this->vw[0], this->vw[1]);
    }
}
void Controller::setup()
{
    ROS_INFO("Setup");
}
void Controller::loopOnce()
{
    auto op_ptr = this->get_joy_signal();
    this->op = this->decode_opcode(op_ptr);
    this->decode_drive(op_ptr);
    #if CONTORLLER_VERBOSE
    stringstream ss;
    std_msgs::String msg;
    ss << "mode:" << (int)this->mode;
    msg.data = ss.str();
    (this->debugger).publish(msg);
    #endif
    if(this->op == Opcode::OPCODE_POWEROFF){
        this->log();
    }
    else if(this->op == Opcode::OPCODE_IDLE){
        ROS_WARN("Direct forced switch to Idle mode");
        this->mode = Mode::MODE_IDLE;
        return;
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
}
void Controller::log()
{
    ROS_INFO("Write log, not implemented yet");
}
void Controller::idle()
{
    set<Opcode> allowed_ops = {
        Opcode::OPCODE_NONE,
        Opcode::OPCODE_HOMEING,
        Opcode::OPCODE_TRAIN_BEGIN,
        Opcode::OPCODE_WORK_BEGIN,
    };
    if(allowed_ops.find(this->op) != allowed_ops.end()){
        //bug
        if(this->op != Opcode::OPCODE_NONE)
            this->mode = pybot::op_to_mode[this->op];
    }
    else{
        ROS_INFO("In Idle mode, %c is not allowed", (char)this->op);
    }
    // ROS_INFO("In Idle mode");
    this->drive();
}
void Controller::clear()
{
    this->is_origin_set = false;
    this->is_calibed = false;
    this->is_calib_begin = false;
    this->is_trained = false;
    ROS_INFO("Clear data, Not implemented yet");
}
void Controller::homing()
{
    // ROS_INFO("In Homing mode");
    set<Opcode> allowed_ops = {
        Opcode::OPCODE_NONE,
        Opcode::OPCODE_ORIGIN,
        Opcode::OPCODE_CALIB_BEGIN,
        Opcode::OPCODE_CALIB_FINISH,
    };
    if(allowed_ops.find(this->op) != allowed_ops.end()){
        if(op == Opcode::OPCODE_ORIGIN){
            this->clear();
            this->is_origin_set = true;
            ROS_INFO("Origin set");
        }
        else if(this->is_origin_set && op == Opcode::OPCODE_CALIB_BEGIN){
            this->is_calib_begin = true;
            ROS_INFO("Calibration begin");
        }
        else if(this->is_calib_begin && op == Opcode::OPCODE_CALIB_FINISH){
            this->mode = Mode(Mode::MODE_IDLE); // bug
            ROS_INFO("Calibration finish, switch to Idle mode");
            this->is_calibed = true;
            this->is_calib_begin = false;
        }
    }
    else{
        ROS_INFO("In Homing mode, %c is not allowed", (char)this->op);
    }
    this->drive();
}
void Controller::training()
{
    // ROS_INFO("In Training mode");
    set<Opcode> allowed_ops = {
        Opcode::OPCODE_NONE,
        Opcode::OPCODE_SETNODE,
        Opcode::OPCODE_TRAIN_FINISH,  
    };
    if(allowed_ops.find(this->op) != allowed_ops.end()){
        if(this->op == Opcode::OPCODE_SETNODE){
            // this->set_node();
            RobotInvoke srv = this->uart.invoke((char)Opcode::OPCODE_SETNODE, vector<int16_t>());
            if(srv.response.is_activated && srv.response.is_arg_valid){
                this->set_node(srv.response.feedback[0]);
            }
            else{
                ROS_ERROR("Invoke %c failed with is_activated = %d, is_arg_valid = %d",
                            srv.request.operation,
                            srv.response.is_activated,
                            srv.response.is_arg_valid);
            }
        }
        else if(this->op == Opcode::OPCODE_TRAIN_FINISH){
            this->mode = Mode::MODE_IDLE;
            this->is_trained = true;
            ROS_INFO("Training finished, Switch to Idle mode");
        }
    }
    else{
        ROS_INFO("In Training mode, %c is not allowed", (char)this->op);
    }
    this->drive();
}
void Controller::set_node(int16_t _node)
{
    ROS_INFO("Set RouteNode on R%dN%d", this->training_route, _node);
}
void Controller::working()
{
    ROS_INFO("In Working mode, not implemented yet");
    switch(this->op){
        case Opcode::OPCODE_WORK_BEGIN:

        case Opcode::OPCODE_TARGET_ROUTE_U:
        case Opcode::OPCODE_TARGET_ROUTE_D:
        case Opcode::OPCODE_TARGET_NODE_U:
        case Opcode::OPCODE_TARGET_NODE_D:
        default:
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
        ROS_INFO("Msg receiced");
        // this->mode = this->mode_tf[_msg->now_mode]; // convert int to char
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
        ROS_WARN("Msg invalid");
    }
}