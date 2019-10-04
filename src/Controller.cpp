#include "Controller.h"
Controller::Controller(const string& _id):
base_driver{UART(_id)}
,joystick{Joystick(_id)}
,wifi{Wifi{_id}}
,tracking_status_sub{this->n.subscribe(ROBOTSTATUS_TOPIC, MSG_QUE_SIZE, &Controller::status_tracking, this)}
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
    }
    return ret;
}
pair<int, int> Controller::decode_drive(sensor_msgs::Joy::ConstPtr& _ptr)
{
    if(_ptr){
        vector<float> axes = _ptr->axes;
        vector<int32_t> buttons = _ptr->buttons;
        // privileged instructions 
        if(buttons[JOYBUTTON_RT]){
            return {0, 0};
        }
        // decode vw
        if(axes[JOYAXES_CROSS_UD] != 0 || axes[JOYAXES_CROSS_LR] != 0){
            int v = axes[JOYAXES_CROSS_UD] * MOTOR_LINEAR_LIMIT / 2;
            int w = axes[JOYAXES_CROSS_LR] * MOTOR_ANGULAR_LIMIT / 2;
            v = (w == 0) ? v : 0;
            // this->pose_tracer.set_vw(v, w);
            return {v, w};
        }
    }
    return {this->pose_tracer.get_v(), this->pose_tracer.get_w()};
}
void Controller::drive()
{
    auto vel_cmd = this->decode_drive(this->op_ptr);
    if(vel_cmd.first != this->pose_tracer.get_v() || vel_cmd.second != this->pose_tracer.get_w()){
        auto srv = this->base_driver.invoke((char)Opcode::OPCODE_DRIVE, {vel_cmd.first, vel_cmd.second});
        #if CONTROLLER_TEST
            this->pose_tracer.set_vw(vel_cmd.first, vel_cmd.second);
        #else
            if(this->base_driver.is_invoke_valid(srv)){
                this->pose_tracer.set_vw(vel_cmd.first, vel_cmd.second);
            }
        #endif
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
    for(int i = 0; i < 4; i++){
        if(this->lidar_levels[i] <= LIDAR_LEVEL_H){
            ROS_WARN("Dir %d near an obstacle", i);
        }
    }
}
void Controller::monitor_display()
{
    stringstream ss;
    char buff[100];
    auto coor = this->pose_tracer.get_coor();
    sprintf(buff, "mode:%d | trk:%d | p:(%+5.1f,%+5.1f,%+5.1f) | v:<%+2d,%+2d> | L:[%d,%d,%d,%d]",
                (int)this->mode,
                (int)this->tracking_status,
                coor.x, coor.y, coor.w,
                this->pose_tracer.get_v(), this->pose_tracer.get_w(),
                this->lidar_levels[0], this->lidar_levels[1], this->lidar_levels[2], this->lidar_levels[2]
                );
    std_msgs::String msg;
    msg.data = string(buff);
    this->monitor.publish(msg);
}
void Controller::loopOnce()
{    
    this->op_ptr = this->get_joy_signal();
    this->op = this->decode_opcode(op_ptr);
    // routine
    this->check_safety();
    // wifi implementation !

    // decode privileged instructions
    switch(this->op){
        // return statements
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
    this->op_ptr = nullptr;
    this->op = Opcode::OPCODE_NONE;
    #if CONTROLLER_VERBOSE
        this->monitor_display();
    #endif
}
void Controller::log()
{
    ROS_INFO("Write log, not implemented yet");
    this->is_ok = false;
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
    this->is_origin_set = false;
    this->is_calibed = false;
    this->is_calib_begin = false;
    this->is_trained = false;
    this->is_ok = true;
    this->pose_tracer.clear();
    this->graph.clear();
    #if CONTROLLER_TEST
        route_ct = node_ct = 0;
    #endif
    ROS_INFO("Clear data, check");
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
                this->pose_tracer.clear();
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
        case Opcode::OPCODE_SETNODE:{
            srv = this->base_driver.invoke((char)Opcode::OPCODE_SETNODE, vector<int16_t>());
            wifi::RouteNode nd;
            #if CONTROLLER_TEST
                srv.response.feedback = vector<int16_t>(1, node_ct++);
                nd.route = this->training_route, nd.node = srv.response.feedback[0], nd.pos = this->pose_tracer.get_coor();
                this->graph.add_node(nd, {nd, this->pose_tracer.get_path()});
                this->pose_tracer.reset();
            #else
                if(this->base_driver.is_invoke_valid(srv)){
                    this->graph.add_node(nd, {nd, this->pose_tracer.get_path()});
                }
                else{
                    ROS_ERROR("SetNode Invokation Ignored");
                }
            #endif
            break;
        }
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
void Controller::working()
{
    switch(this->op){

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