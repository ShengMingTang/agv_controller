#include "Controller.h"
Controller::Controller(const string& _id):
base_driver{UART(_id)}
,joystick{Joystick(_id)}
// ,wifi{Wifi{_id}}
,tracking_status_sub{this->n.subscribe(ROBOTSTATUS_TOPIC, MSG_QUE_SIZE, &Controller::status_tracking, this)}
,monitor{this->n.advertise<std_msgs::String>(MONITOR_TOPIC, MSG_QUE_SIZE)}
,lidar_levels {vector<int16_t>(4, LIDAR_LEVEL_L)}

,nodeocp_srv{this->n.advertiseService(ROBOT_WIFI_NODEOCP_INNER, &Controller::nodeocp_serve, this)}
,nodeocp_clt{this->n.serviceClient<tircgo_msgs::WifiNodeOcp>(ROBOT_WIFI_NODEOCP_OUTER)}
,nodecost_srv{this->n.advertiseService(ROBOT_WIFI_NODECOST_INNER, &Controller::nodecost_serve, this)}
,nodecost_clt{this->n.serviceClient<tircgo_msgs::WifiNodeOcp>(ROBOT_WIFI_NODECOST_OUTER)}

,task_confirm_srv{this->n.advertiseService(ROBOT_WIFI_TASK_CONFIRM_INNER, &Controller::task_confirm_serve, this)}

,askdata_srv(this->n.advertiseService(ROBOT_ASKDATA_TOPIC, &Controller::askdata_serve, this))

{
    this->nd_ocp.route = this->nd_ocp.node = -1;
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
    // can wait for UART in the future
}
void Controller::isr(const Tracking_status& _cond)
{
    // implement
    ROS_INFO("Encounter some obstacle or out of track");
}

/* Drive related */
/* decode opcode according to state */
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
                #if AGV_CONTROLLER_WORK_MAN
                    else if(buttons[JOYBUTTON_RT]){
                        this->nd_target.route++;
                    }
                    else if(buttons[JOYBUTTON_LT]){
                        if(this->nd_target.route - 1 >= 0)
                            this->nd_target.route--;
                    }
                    else if(buttons[JOYBUTTON_RB]){
                        this->nd_target.node++;
                    }
                    else if(buttons[JOYBUTTON_LB]){
                        if(this->nd_target.node - 1 >= 0)
                            this->nd_target.node--;
                    }
                #endif
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
                if(buttons[JOYBUTTON_B])
                    ret = Opcode::OPCODE_WORK_FINISH;
                break;
            default:
                break;
        }
    }
    return ret;
}
/* decode joystick to drive signal */
pair<int16_t, int16_t> Controller::decode_drive(sensor_msgs::Joy::ConstPtr& _ptr)
{
    if(_ptr){
        vector<float> axes = _ptr->axes;
        vector<int32_t> buttons = _ptr->buttons;
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
    // empty
    return {this->pose_tracer.get_v(), this->pose_tracer.get_w()};
}
/* drive machine interface */
void Controller::drive()
{
    auto vel_cmd = this->decode_drive(this->op_ptr);
    double t = (ros::Time::now() - this->pose_tracer.get_starttime()).toSec();
    if(t >= AGV_CONTROLLER_DRIVE_TIMEOUT || vel_cmd.first != this->pose_tracer.get_v() || vel_cmd.second != this->pose_tracer.get_w()){ // different motion
        if(vel_cmd.first == 0 && this->pose_tracer.get_v() == 0 &&
           vel_cmd.second == 0 && this->pose_tracer.get_w() == 0){
               // don't send redundant 0 vel_cmd
        }
        else{
            #if AGV_CONTROLLER_SAFE
                if(this->check_safety()){
                    auto srv = this->base_driver.invoke((char)Opcode::OPCODE_DRIVE, {vel_cmd.first, vel_cmd.second});    
                }
            #else
                auto srv = this->base_driver.invoke((char)Opcode::OPCODE_DRIVE, {vel_cmd.first, vel_cmd.second});
            #endif
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
}
/* Sys related */
void Controller::clear()
{
    this->is_origin_set = false;
    this->is_calibed = false;
    this->is_trained = false;
    this->is_calib_begin = false;
    this->is_ok = true;
    this->op_ptr = nullptr;
    this->op = Opcode::OPCODE_NONE;
    this->pose_tracer.clear();
    this->graph.clear();
    #if AGV_CONTROLLER_TEST
        route_ct = node_ct = 0;
    #endif
    ROS_INFO("Clear data, not implemented yet");
}
/* writing log files */
void Controller::log()
{
    ROS_INFO("Write log, not implemented yet");
}
/* safety issue */
bool Controller::check_safety()
{
    return this->lidar_levels[LIDAR_DIR_FRONT] <= LIDAR_LEVEL_H;
}
/* subsriber to track status */
void Controller::status_tracking(const RobotStatus::ConstPtr& _msg)
{
    if(_msg->is_activated){
        #if !AGV_CONTROLLER_TEST
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
void Controller::monitor_display()
{
    stringstream ss;
    char buff[100];
    auto coor = this->pose_tracer.get_coor();
    sprintf(buff, "mode:%d| trk:%d| p:(%+5.1f,%+5.1f,%+5.1f)| v:<%+2d,%+2d>| L:[%d,%d,%d,%d]| T<%d,%d>",
                (int)this->mode,
                (int)this->tracking_status,
                coor.x, coor.y, coor.w,
                this->pose_tracer.get_v(), this->pose_tracer.get_w(),
                this->lidar_levels[0], this->lidar_levels[1], this->lidar_levels[2], this->lidar_levels[2],
                this->nd_target.route, this->nd_target.node
                );
    std_msgs::String msg;
    msg.data = string(buff);
    this->monitor.publish(msg);
}
bool Controller::is_target_ocp(const Node& _target)
{
    // request a service from Ng
    tircgo_msgs::WifiNodeOcp srv;
    if(!this->nodeocp_clt.call(srv)){
        ROS_ERROR("<NodeOcp Srv-Err>");
        return false;
    }
    return srv.response.error_code == WIFI_ERR_NONE && srv.response.is_ocp;
}