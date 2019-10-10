#include "Controller.h"
Controller::Controller(const string& _id):
base_driver{UART(_id)}
,joystick{Joystick(_id)}
// ,wifi{Wifi{_id}}
,tracking_status_sub{this->n.subscribe(ROBOTSTATUS_TOPIC, MSG_QUE_SIZE, &Controller::status_tracking, this)}
,monitor{this->n.advertise<std_msgs::String>(MONITOR_TOPIC, MSG_QUE_SIZE)}
,lidar_levels {vector<int16_t>(4, LIDAR_LEVEL_L)}
{
    ROS_INFO("Controller constructed");
}
Controller::~Controller()
{
    ROS_INFO("Controller destructed");
}
void Controller::setup()
{
    ROS_INFO("Controller Setup:");
    ROS_INFO("Loop at frequency %d", AGV_LOOP_FREQ);
    ROS_INFO("Verbose : %d", AGV_CONTROLLER_VERBOSE);
    ROS_INFO("UART dominates statusc : %d", AGV_CONTROLLER_UART_DOMIN);
}
void Controller::loopOnce()
{
    RobotInvoke srv;
    this->op_ptr = this->get_joy_signal();
    this->op = this->decode_opcode(op_ptr);
    // routine
    this->check_safety();
    // wifi implementation !

    // decode privileged instructions
    switch(this->op){
        // return statements
        case Opcode::OPCODE_POWEROFF:
            srv = this->base_driver.invoke((char)Opcode::OPCODE_POWEROFF, vector<int16_t>());
            #if AGV_CONTROLLER_TEST
                this->log();
                this->is_ok = false;
                ROS_INFO("Power off!");
            #else
                if(this->base_driver.is_invoke_valid(srv)){
                    this->log();
                    this->is_ok = false;
                    ROS_INFO("Power off!");
                }
                else{
                    ROS_ERROR("<Poweroff Srv-Err>");
                }
            #endif
            break;
        default:
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
            break;
    }
    this->op_ptr = nullptr;
    this->op = Opcode::OPCODE_NONE;
    #if AGV_CONTROLLER_VERBOSE
        this->monitor_display();
    #endif
}
void Controller::idle()
{
    RobotInvoke srv;
    switch(this->op){
        case Opcode::OPCODE_NONE:
            break;
        case Opcode::OPCODE_HOMEING:
            srv = this->base_driver.invoke((char)Opcode::OPCODE_HOMEING, vector<int16_t>());
            #if AGV_CONTROLLER_TEST
                this->mode = Mode::MODE_HOMING;
                ROS_INFO("Start Homing");
            #else
                if(this->base_driver.is_invoke_valid(srv)){
                    this->clear();
                    ROS_INFO("Start Homing");
                }
                else{
                    ROS_ERROR("<Homing Srv-Err>");
                }
            #endif
            break;
        case Opcode::OPCODE_TRAIN_BEGIN:
            srv = this->base_driver.invoke((char)Opcode::OPCODE_TRAIN_BEGIN, vector<int16_t>());
            #if AGV_CONTROLLER_TEST
                this->mode = Mode::MODE_TRAINING;
                node_ct = 0;
                this->training_route = route_ct++;
                ROS_INFO("Start Training @ R%d", this->training_route);
            #else
                if(this->base_driver.is_invoke_valid(srv)){
                    this->training_route = srv.response.feedback[0];
                    ROS_INFO("Start Training @ R%d", this->training_route);
                }
                else{
                    ROS_ERROR("<Traing begin Srv-Err>");
                }
            #endif
            break;
        case Opcode::OPCODE_WORK_BEGIN:
            srv = this->base_driver.invoke((char)Opcode::OPCODE_WORK_BEGIN, vector<int16_t>());
            this->working_route = this->working_node = 0;
            #if AGV_CONTROLLER_TEST
                this->mode = Mode::MODE_WORKING;
                ROS_INFO("Start Working !!");
            #else
                if(this->base_driver.is_invoke_valid(srv)){
                    ROS_INFO("Start Working !! Not implemented");
                }
                else{
                    ROS_ERROR("<Working begin Srv-Err>");
                }
            #endif
            break;
        default:
            ROS_WARN("In Idle mode, %c is not allowed", (char)this->op);
            break;
    }
    this->drive();
}
void Controller::homing()
{
    RobotInvoke srv;
    switch(this->op){
        case Opcode::OPCODE_NONE:
            break;
        case Opcode::OPCODE_ORIGIN:
            if(!this->is_calib_begin){
                srv = this->base_driver.invoke((char)Opcode::OPCODE_ORIGIN, vector<int16_t>());
                #if AGV_CONTROLLER_TEST
                    this->pose_tracer.clear();
                    this->is_origin_set = true;
                    ROS_INFO("Origin set");
                #else
                    if(this->base_driver.is_invoke_valid(srv)){\
                        this->pose_tracer.clear();
                        ROS_INFO("Origin set");
                        this->is_origin_set = true;
                    }
                    else{
                        ROS_ERROR("<Origin set Srv-Err>");
                    }
                #endif
            }
            break;
        case Opcode::OPCODE_CALIB_BEGIN:
            if(this->is_origin_set && !this->is_calib_begin){
                srv = this->base_driver.invoke((char)Opcode::OPCODE_CALIB_BEGIN, vector<int16_t>());
                #if AGV_CONTROLLER_TEST
                    this->is_calib_begin = true;
                    ROS_INFO("Calib begin");
                #else
                    if(this->base_driver.is_invoke_valid(srv)){
                        this->is_calib_begin = true;
                        ROS_INFO("Calibration begin");
                    }
                    else{
                        ROS_ERROR("<Calib begin Srv-Err>");
                    }
                #endif
            }
            else{
                if(!this->is_origin_set)
                    ROS_ERROR("[Calib begin Err], Set Origin first plz");
                else if(this->is_calib_begin)
                    ROS_ERROR("[Calib begin Err], calib already started");
            }
            break;
        case Opcode::OPCODE_CALIB_FINISH:
            if(this->is_calib_begin){
                this->is_calibed = true;
                this->is_calib_begin = false;
                srv = this->base_driver.invoke((char)Opcode::OPCODE_CALIB_FINISH, vector<int16_t>());
                #if AGV_CONTROLLER_TEST
                    ROS_INFO("Calib finish");
                    this->mode = Mode(Mode::MODE_IDLE);
                #else
                    if(this->base_driver.is_invoke_valid(srv)){
                        ROS_INFO("Calib finish, switch to Idle mode");
                    }
                    else{
                        ROS_ERROR("<Calib_finish Srv-Err>");
                    }
                #endif
            }
            else{
                ROS_ERROR("[Calib finish Err], Calib_begin not executed");
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
    geometry_msgs::Quaternion pos;
    // node
    switch(this->op){
        case Opcode::OPCODE_NONE:
            break;
        case Opcode::OPCODE_SETNODE:
            srv = this->base_driver.invoke((char)Opcode::OPCODE_SETNODE, vector<int16_t>());
            #if AGV_CONTROLLER_TEST
                srv.response.feedback = vector<int16_t>(1, node_ct++);
                // graph routine
                pos = this->pose_tracer.get_coor();
                this->pose_tracer.reset_path();
                ROS_INFO("Node R%d, N%d @ (%f,%f)", this->training_route, node_ct, pos.x, pos.y);
            #else
                if(this->base_driver.is_invoke_valid(srv)){
                    // graph routine
                    pos = this->pose_tracer.get_coor();
                    this->pose_tracer.reset_path();
                    ROS_INFO("Node R%d, N%d @ (%f,%f)", this->training_route, srv.response.feedback[0], pos.x, pos.y);
                }
                else{
                    ROS_ERROR("<SetNode Srv-Err>");
                }
            #endif
            break;
        case Opcode::OPCODE_TRAIN_FINISH:
            this->is_trained = true;
            srv = this->base_driver.invoke((char)Opcode::OPCODE_TRAIN_FINISH, vector<int16_t>());
            #if AGV_CONTROLLER_TEST
                this->mode = Mode::MODE_IDLE;
                ROS_INFO("Training finished");
            #else
                if(this->base_driver.is_invoke_valid(srv)){
                    ROS_INFO("Training finished");
                }
                else{
                    ROS_ERROR("<Traing finished Srv-Err>");
                }
            #endif
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
            ROS_WARN("Workin mode not implemented");
            break;
    }
}
/* Drive related */
Opcode Controller::decode_opcode(sensor_msgs::Joy::ConstPtr& _ptr)
{
    Opcode ret = Opcode::OPCODE_NONE;
    if(_ptr){
        vector<float> axes = _ptr->axes;
        vector<int32_t> buttons = _ptr->buttons;
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
                else if(buttons[JOYBUTTON_RT])
                    ret = Opcode::OPCODE_WORK_FINISH;
        }
    }
    return ret;
}
pair<int16_t, int16_t> Controller::decode_drive(sensor_msgs::Joy::ConstPtr& _ptr)
{
    if(_ptr){
        vector<float> axes = _ptr->axes;
        vector<int32_t> buttons = _ptr->buttons;
        // privileged instructions 
        if(buttons[JOYBUTTON_LT]){
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
    // empty
    return {this->pose_tracer.get_v(), this->pose_tracer.get_w()};
}
void Controller::drive()
{
    auto vel_cmd = this->decode_drive(this->op_ptr);
    double t = (ros::Time::now() - this->pose_tracer.get_starttime()).toSec();
    if(t >= AGV_CONTROLLER_DRIVE_TIMEOUT || vel_cmd.first != this->pose_tracer.get_v() || vel_cmd.second != this->pose_tracer.get_w()){ // different motion
        // ROS_INFO("%d, %d   %d,%d", vel_cmd.first, vel_cmd.second, this->pose_tracer.get_v(), this->pose_tracer.get_w());
        auto srv = this->base_driver.invoke((char)Opcode::OPCODE_DRIVE, {vel_cmd.first, vel_cmd.second});
        #if AGV_CONTROLLER_TEST
            this->pose_tracer.set_vw(vel_cmd.first, vel_cmd.second);
        #else
            if(this->base_driver.is_invoke_valid(srv)){
                this->pose_tracer.set_vw(vel_cmd.first, vel_cmd.second);
            }
            else{
                ROS_ERROR("<Drive Srv-Err>");
            }
        #endif
    }
}
/* Sys related */
void Controller::clear()
{
    this->is_origin_set = false;
    this->is_calibed = false;
    this->is_calib_begin = false;
    this->is_trained = false;
    this->is_ok = true;
    this->pose_tracer.clear();
    // this->graph.clear();
    #if AGV_CONTROLLER_TEST
        route_ct = node_ct = 0;
    #endif
    ROS_INFO("Clear data, not implemented yet");
}
void Controller::log()
{
    ROS_INFO("Write log, not implemented yet");
}
bool Controller::check_safety()
{
    for(int i = 0; i < this->lidar_levels.size(); i++){
        if(this->lidar_levels[i] <= LIDAR_LEVEL_H){
            ROS_WARN("Dir %d near an obstacle", i);
        }
    }
}
void Controller::status_tracking(const RobotStatus::ConstPtr& _msg)
{
    if(_msg->is_activated){
        #if AGV_CONTROLLER_VERBOSE
            ROS_INFO("RobotStatus Msg receiced");
        #endif
        #if AGV_CONTROLLER_UART_DOMIN
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
        #else
            ROS_WARN("Mode, Lidar aren't under UART's control")
        #endif
    }
    else{
        ROS_WARN("RobotStatus not activated");
    }
}
sensor_msgs::Joy::ConstPtr Controller::get_joy_signal()
{
    return this->joystick.pop();
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