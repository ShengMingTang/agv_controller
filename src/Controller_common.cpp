#include "Controller.h"
Controller::Controller(const string& _id):
base_driver{UART(_id)}
,joystick{Joystick(_id)}
,tracking_status_sub{this->n.subscribe(ROBOTSTATUS_TOPIC, MSG_QUE_SIZE, &Controller::status_tracking, this)}
,monitor{this->n.advertise<std_msgs::String>(MONITOR_TOPIC, MSG_QUE_SIZE)}
,lidar_levels {vector<int16_t>(4, LIDAR_LEVEL_FAR)}
,rn_img{vector< vector<VertexType*> >(TRAIN_ROUTE_MAX, vector<VertexType*>() )}

,nodeocp_srv{this->n.advertiseService(ROBOT_WIFI_NODEOCP_INNER, &Controller::nodeocp_serve, this)}
,nodeocp_clt{this->n.serviceClient<tircgo_msgs::WifiNodeOcp>(ROBOT_WIFI_NODEOCP_OUTER)}
,nodecost_srv{this->n.advertiseService(ROBOT_WIFI_NODECOST_INNER, &Controller::nodecost_serve, this)}
,nodecost_clt{this->n.serviceClient<tircgo_msgs::WifiNodeOcp>(ROBOT_WIFI_NODECOST_OUTER)}

,task_confirm_srv{this->n.advertiseService(ROBOT_WIFI_TASK_CONFIRM_INNER, &Controller::task_confirm_serve, this)}

,askdata_srv{this->n.advertiseService(ROBOT_WIFI_ASKDATA_INNER, &Controller::askdata_serve, this)}

// controller space
,sch_srv{this->n, (_id + ROBOT_SCHEDULER_CONTROLLER).c_str(), boost::bind(&Controller::execute_schedule, this, _1), false}
,pub_agent_imm{this->n.advertise<tircgo_msgs::ControllerTalk>((_id + ROBOT_CONTROLLER_AGENT).c_str(), MSG_QUE_SIZE)}

{
    this->nd_training.route = this->nd_training.node = 0;
    this->ocp_vptr = nullptr;
    ROS_INFO("Controller constructed");
}
Controller::~Controller()
{
    ROS_INFO("Controller destructed");
}
void Controller::setup(int _argc, char **argv)
{
    string opt;
    for(int i = 1; i < _argc; i++){
        opt = argv[i];
        if(opt == "-h"){
            ROS_INFO("|-----------------------------------------------|");
            ROS_INFO("| [-h]: display this msg                        |");
            ROS_INFO("| [-i]: string, identifier of this robot        |");
            ROS_INFO("| [-f]: int = 20, operating frequency           |");
            ROS_INFO("| [-d]: float = 0.4, drive timeout              |");
            ROS_INFO("| [-p]: int = 60, precision for merging nodes   |");
            ROS_INFO("| [-m]: int = 0, set control (using bitmap)     |");
            ROS_INFO("| control = {wifi(0/1)}                         |");
            ROS_INFO("|-----------------------------------------------|");
            this->stage_bm |= MODE_NOTOK;
        }
        else if(opt == "-f"){
            if(i + 1 < _argc){
                opt = argv[i+1];
                this->loop_rate = ros::Rate(stoi(opt));
                i++;
            }
            else{
                ROS_ERROR("Invalid number of cmd line args");
            }
        }
        else if(opt == "-d"){
            if(i + 1 < _argc){
                opt = argv[i+1];
                this->drive_timeout = stof(opt);
                i++;
            }
            else{
                ROS_INFO("Invalid number of cmd line args");
            }
        }
        else if(opt == "-p"){
            if(i + 1 < _argc){
                opt = argv[i+1];
                this->close_enough = stof(opt);
                i++;
            }
            else{
                ROS_ERROR("Invalid number of cmd line args");
            }
        }
        else if(opt == "-m"){
            if(i + 1 < _argc){
                opt = argv[i+1];
                this->control = stoi(opt);
                i++;
            }
            else{
                ROS_ERROR("Invalid number of cmd line args");
            }
        }
        else if(opt == "-i"){
            if(i + 1 < _argc){
                opt = argv[i+1];
                this->frame_id = opt;
                i++;
            }
            else{
                ROS_ERROR("Invalid number of cmd line args");
            }
        }
        else{
            ROS_INFO("Unknown option, neglected");
        }
    }
    if(this->ok()){
        ROS_INFO("============================================================");
        ROS_INFO("Controller Setup :");
        ROS_INFO("> Only objects in front/back could block, neglect side objects");
        ROS_INFO("> System parameter :");
        ROS_INFO("> Control mode : %d", this->control);
        ROS_INFO("> Drive Refresh Time : %.1f", this->drive_timeout);
        ROS_INFO("> Node merge max separation : %d", this->close_enough);
        ROS_INFO("============================================================");
        ROS_INFO("Wait for UART");
        RobotInvoke srv = this->base_driver.invoke(OPCODE_SIGNAL, {DEVICE_BEEPER, DEVICE_BEEPER_3L_2S, 1});
        ros::Rate loop(1);
        while(ros::ok() && this->ok() && !(this->base_driver.is_invoke_valid(srv))){
            srv = this->base_driver.invoke(OPCODE_SIGNAL, {DEVICE_BEEPER, DEVICE_BEEPER_3L_2S, 1});
            loop.sleep();
            ros::spinOnce();
        }
        ROS_INFO("UART is ready");
        ROS_INFO("Setup Done");
    }
}
/*
    Drive related
    decode opcode according to state
    Non-state-transition-related op will be decoded as OPCODE_NONE 
    and get executed in place
*/
int16_t Controller::decode_opcode()
{
    int16_t ret = OPCODE_NONE;
    if(this->op_ptr){
        vector<float> axes = this->op_ptr->axes;
        vector<int32_t> buttons = this->op_ptr->buttons;
        if(buttons[JOYBUTTON_BACK]){
            return OPCODE_SHUTDOWN;
        }
        else if(!(this->stage_bm & MODE_AUTO) && buttons[JOYBUTTON_START]){
            return OPCODE_AUTO_BEGIN;
        }
        else if(this->stage_bm & MODE_AUTO && buttons[JOYBUTTON_START]){
            return OPCODE_AUTO_FINISH;
        }
        else if(buttons[JOYBUTTON_LB]){
            // display UART history
            ROS_INFO("%s", this->base_driver.get_cmds().c_str());
        }
        else if(buttons[JOYBUTTON_LT]){
            // clear UART history
            this->base_driver.clear();
            ROS_INFO("Clear UART history");
        }
        else if(buttons[JOYBUTTON_RT]){
            // publish our UART histroy anonymously, all receiver
            tircgo_msgs::ControllerTalk msg;
            msg.talk = this->base_driver.get_cmds();
            this->pub_agent_imm.publish(msg);
            ROS_INFO("Publish UART history");
        }
        else if(buttons[JOYBUTTON_RB]){
            // switch agent action input
            if(this->stage_bm & MODE_AGENT){
                this->stage_bm &= (~MODE_AGENT);
                ROS_INFO("Turn off agent input");
            }
            else{
                this->stage_bm |= MODE_AGENT;
                this->sch_srv.start();
                ROS_INFO("Turn on agent input");
            }
        }
        // decode mode dependent instructions
        switch(this->mode){
            case MODE_IDLE:
                if(buttons[JOYBUTTON_Y])
                    ret = OPCODE_CALIB;
                else if(buttons[JOYBUTTON_X])
                    ret = OPCODE_TRAIN_BEGIN;
                else if(buttons[JOYBUTTON_B])
                    ret = OPCODE_WORK_BEGIN;

                else if(axes[JOYAXES_CROSS_LR] == -1.0)
                    this->nd_target.route = (this->nd_target.route + 1) % TRAIN_ROUTE_MAX;
                else if(axes[JOYAXES_CROSS_LR] == 1 && this->nd_target.route > 0){
                    this->nd_target.route--;
                }
                else if(axes[JOYAXES_CROSS_UD] == 1.0)
                    this->nd_target.node++;
                else if(axes[JOYAXES_CROSS_UD] == -1 && this->nd_target.node > 0){
                    this->nd_target.node--;
                }
                
                else if(buttons[JOYBUTTON_STICK_RIGHT] && this->nd_training.route + 1 < TRAIN_ROUTE_MAX){
                    this->nd_training.route++;
                }
                else if(buttons[JOYBUTTON_STICK_LEFT] && this->nd_training.route > 0){
                    this->nd_training.route--;
                }

                break;
            case MODE_TRAINING:
                if(buttons[JOYBUTTON_X])
                    ret = OPCODE_SETNODE;
                else if(buttons[JOYBUTTON_B])
                    ret = OPCODE_TRAIN_FINISH;
                break;
            case MODE_WORKING:
                if(buttons[JOYBUTTON_A])
                    ret = OPCODE_WORK_FINISH;
                break;
            default:
                break;
        }
    }
    return ret;
}

/*
    decode joystick to drive signal
    independent of int16_t decoding
*/
vector<int16_t> Controller::decode_drive()
{
    if(this->op_ptr){
        vector<float> axes = this->op_ptr->axes;
        vector<int32_t> buttons = this->op_ptr->buttons;
        if(abs(axes[JOYAXES_STICKLEFT_UD]) > abs(axes[JOYAXES_STICKLEFT_LR])){
            return {static_cast<int16_t>(axes[JOYAXES_STICKLEFT_UD]) * static_cast<int16_t>(DRIVE_VEL_LINEAR), 0};
        }
        else{
            return {0, static_cast<int16_t>(axes[JOYAXES_STICKLEFT_LR]) * static_cast<int16_t>(DRIVE_VEL_ANGULAR)};
        }
    }
    return this->pose_tracer.get_vel();
}

/* 
    drive machine interface, locked if trained (not implemented)
    Call this only in Idle, Homing, Training
*/
bool Controller::drive(vector<int16_t> _vel)
{
    // trained but not in training
    if((this->stage_bm & MODE_TRAINING) && (this->mode != MODE_TRAINING || this->nd_training.node == -1)){
        _vel = {0, 0};
    }
    this->op_vel = _vel;
    // // legal condition to driveif(this->check_safety()){
    auto srv = this->base_driver.invoke(OPCODE_DRIVE, _vel);
    if(this->base_driver.is_invoke_valid(srv)){
        this->pose_tracer.set_vw(_vel);
    }
    else{
        ROS_INFO("Unsafe condition, drive rejected");
        return false;
    }

    return true;
}

/* Sys related */
void Controller::clear()
{
    this->stage_bm = 0;
    this->runtime_vars_mgr(RUNTIME_VARS_RESET);
    this->pose_tracer.clear();
    this->graph.clear();
    for(auto &it : this->rn_img){
        it.clear();
    }
    this->work_list.clear();
    // this->base_driver.clear();
    ROS_INFO("Clear data, UART histroy not cleared, press LT to clear");
    auto srv = this->base_driver.invoke(OPCODE_SIGNAL, {DEVICE_BEEPER, DEVICE_BEEPER_3L_2S, 1});
    this->base_driver.is_invoke_valid(srv);
}

/* writing log files */
void Controller::log()
{
    ROS_ERROR("Write log, should implment serialization");
}

/* shutdown routine */
bool Controller::shutdown()
{
    auto srv = this->base_driver.invoke(OPCODE_SHUTDOWN, vector<int16_t>());
    if(this->base_driver.is_invoke_valid(srv)){
        this->log();
        this->stage_bm |= MODE_NOTOK;
        ROS_INFO("Controller Shutdown!");
    }
}
/* safety issue */
bool Controller::check_safety()
{
    bool ret;
    if(this->mode != MODE_WORKING){
        if(this->op_vel[0] > 0)
            return this->lidar_levels[LIDAR_DIR_FRONT] < LIDAR_LEVEL_CLOSE;
        else if(this->op_vel[0] < 0)
            return this->lidar_levels[LIDAR_DIR_BACK] < LIDAR_LEVEL_CLOSE;
        else
            ret = true;
    }
    else{
        ret = this->lidar_levels[LIDAR_DIR_FRONT] < LIDAR_LEVEL_CLOSE &&
            this->lidar_levels[LIDAR_DIR_BACK] < LIDAR_LEVEL_CLOSE; 
    }
    if(!ret){
        ROS_WARN("Near an obstacle");
        this->base_driver.invoke(DEVICE_LED_R, {DEVICE_LED_ON, 1});
    }
    return ret;
}


/* interface for buffering joystick signal */
sensor_msgs::Joy::ConstPtr Controller::get_joy_signal()
{
    return this->joystick.pop();
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
        this->op = OPCODE_NONE;
    }
}
bool Controller::priviledged_instr()
{
    bool ret = true;
    switch(this->op){
        // return statements
        case OPCODE_SHUTDOWN:
            this->shutdown();
            break;
        case OPCODE_AUTO_BEGIN:
            if(this->stage_bm & MODE_TRAINING){
                this->stage_bm |= MODE_AUTO;
                this->sch_srv.start();
                ROS_WARN("Start auto mode");
            }
            else{
                ROS_ERROR("Not trained, press auto has no effect");
            }
            break;
        case OPCODE_AUTO_FINISH:
            this->stage_bm &= (~MODE_AUTO);
            ROS_WARN("Finish auto mode, script has no effect now");
        default:
            ret = false;
            break;
    }
    return ret;
}
/* return if the _route, _node pair is valid in the current robot */
bool Controller::is_target_valid(const RouteNode &_nd)
{
    if(_nd.route >= 0 && _nd.route < this->rn_img.size()){
        return _nd.node < this->rn_img[_nd.route].size(); // short check, assume nodes come in successive order
    }
    return false;
}