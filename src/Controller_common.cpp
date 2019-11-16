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
    ROS_WARN("Only objects in front/back could block, neglect side objects");
    #if !ROBOT_CONTROLLER_TEST
    RobotInvoke srv;
    ROS_INFO("Wait for UART");
    do{
        srv = this->base_driver.invoke(Opcode::OPCODE_SIGNAL, {DEVICE_BEEPER, DEVICE_BEEPER_3L_2S});
    }while(!(this->base_driver.is_invoke_valid(srv)));
    #endif
    ROS_INFO("Setup Done");
}
void Controller::isr(const int _inter)
{
    switch (_inter)
    {
    case ISR_OBSTACLE:{
        auto srv = this->base_driver.invoke(Opcode::OPCODE_SIGNAL, {DEVICE_BEEPER, DEVICE_BEEPER_2S});
        #if ROBOT_CONTROLLER_TEST
            ROS_INFO("Calling ISR_OBSTAVLE (Test)");
        #else
            if(!(this->base_driver.is_invoke_valid(srv))){
                ROS_ERROR("<ISR_OBS Srv Err>");
            }
        #endif
    }
        break;
    default:
        break;
    }
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
            return Opcode::OPCODE_SHUTDOWN;        
        // decode mode dependent instructions
        switch(this->mode){
            case MODE_IDLE:
                if(buttons[JOYBUTTON_Y])
                    ret = Opcode::OPCODE_CALIB;
                else if(buttons[JOYBUTTON_X])
                    ret = Opcode::OPCODE_TRAIN_BEGIN;
                else if(buttons[JOYBUTTON_B])
                    ret = Opcode::OPCODE_WORK_BEGIN;

                else if(buttons[JOYBUTTON_RT])
                    this->nd_target.route = (this->nd_target.route + 1) % TRAIN_ROUTE_MAX;
                else if(buttons[JOYBUTTON_LT] && this->nd_target.route > 0){
                    this->nd_target.route--;
                }
                else if(buttons[JOYBUTTON_RB])
                    this->nd_target.node++;
                else if(buttons[JOYBUTTON_LB] && this->nd_target.node > 0){
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
                    ret = Opcode::OPCODE_SETNODE;
                else if(buttons[JOYBUTTON_B])
                    ret = Opcode::OPCODE_TRAIN_FINISH;
                break;
            case MODE_WORKING:
                if(buttons[JOYBUTTON_A])
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
            int16_t vv = axes[JOYAXES_CROSS_UD] * DRIVE_VEL_LINEAR;
            int16_t ww = axes[JOYAXES_CROSS_LR] * DRIVE_VEL_ANGULAR;
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
bool Controller::drive(vector<int16_t> _vel)
{
    this->op_vel = _vel;
    // trained but not in training
    if((this->stage_bm & MODE_TRAINING) && this->mode != MODE_TRAINING)
        return false;
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
                    auto srv = this->base_driver.invoke(Opcode::OPCODE_DRIVE, {_vel[0], _vel[1]});
                    if(this->base_driver.is_invoke_valid(srv)){
                        this->pose_tracer.set_vw(_vel);
                    }
                    else{
                        ROS_ERROR("<Drive Srv-Err>");
                    }

                }
                else{
                    ROS_INFO("Unsafe condition, drive rejected");
                    return false;
                }
            #endif
        }
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
    // reset all runtime RouteNodes
    #if ROBOT_CONTROLLER_TEST
        node_ct = 0;
    #endif
    ROS_INFO("Clear data");
}

/* writing log files */
void Controller::log()
{
    ROS_INFO("Write log, should implment serialization");
}

/* shutdown routine */
bool Controller::shutdown()
{
    auto srv = this->base_driver.invoke(Opcode::OPCODE_SHUTDOWN, vector<int16_t>());
    #if ROBOT_CONTROLLER_TEST
        this->log();
        this->stage_bm |= MODE_NOTOK;
        ROS_INFO("Controller Shutdown! (Test)");
    #else
    if(this->base_driver.is_invoke_valid(srv)){
        this->log();
        this->stage_bm |= MODE_NOTOK;
        ROS_INFO("Controller Shutdown!");
    }
    else{
        ROS_ERROR("<Shutdownroff Srv-Err>");
    }
    #endif
}
/* safety issue */
bool Controller::check_safety()
{
    if(this->mode != MODE_WORKING){
        if(this->op_vel[0] > 0)
            return this->lidar_levels[LIDAR_DIR_FRONT] <= LIDAR_LEVEL_H;
        else if(this->op_vel[0] < 0)
            return this->lidar_levels[LIDAR_DIR_BACK] <= LIDAR_LEVEL_H;
        else
            return true;
    }
    else{
        return this->lidar_levels[LIDAR_DIR_FRONT] <= LIDAR_LEVEL_H &&
            this->lidar_levels[LIDAR_DIR_BACK] <= LIDAR_LEVEL_H; 
    }
}
/* subsriber to track status */
void Controller::status_tracking(const RobotStatus::ConstPtr& _msg)
{
    if(_msg->is_activated){
        #if !ROBOT_CONTROLLER_TEST
            this->mode = _msg->now_mode;
            this->stage_bm |= _msg->now_mode;
            if(_msg->tracking_status_reply.is_activated){
                this->tracking_status = (Tracking_status)_msg->tracking_status_reply.reply;
            }
            else{
                ROS_WARN("Tracking_status_reply invalid");
            }
            if(_msg->lidar_level_reply.is_activated){
                this->lidar_levels = _msg->lidar_level_reply.level_reply;
            }
            else{
                ROS_WARN("Lidar_level_reply invalid");
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
    char buff[150];
    auto coor = this->pose_tracer.get_coor();
    sprintf(buff, "mode:%d, trk:%d, pos:(%+.1f,%+.1f %+.1f), v:<%+2d,%+2d>, L:[%d,%d,%d,%d], Trn{%d, %d}, Tar<%d,%d>",
                (int)this->mode,
                (int)this->tracking_status,
                coor.x, coor.y, (coor.w) / 3.14159 * 180,
                this->pose_tracer.get_vel()[0], this->pose_tracer.get_vel()[1],
                this->lidar_levels[0], this->lidar_levels[1], this->lidar_levels[2], this->lidar_levels[2],
                this->nd_training.route, this->nd_training.node,
                this->nd_target.route, this->nd_target.node
                );
    std_msgs::String msg;
    msg.data = string(buff);
    this->monitor.publish(msg);
}

/* dumps graph to string*/
string Controller::dumps_graph()
{
    auto coor = this->pose_tracer.get_coor(); 
    geometry_msgs::Point pos;
    pos.x = coor.x, pos.y = coor.y;
    string digraph;
    digraph += "graph G {\n";
    digraph += "rankdir = LR;\n";
    digraph += "#Tircgo[pos = \"" + to_string(coor.x) + "," + to_string(coor.y) + "!\"]\n";
    // subgraph
    int subgraph_count = 0;
    double max_x = 0, max_y = 0;
    for(auto it : this->graph.vertices){
        if(abs(it.pos.x) > max_x)
            max_x = abs(it.pos.x);
        if(abs(it.pos.y) > max_y)
            max_y = abs(it.pos.y);
    }
    if(abs(coor.x) > max_x)
        max_x = abs(coor.x);
    if(abs(coor.y) > max_y)
        max_y = abs(coor.y);
    if(max_x == 0)
        max_x = 1;
    if(max_y == 0)
        max_y = 1;
    for(auto it : this->graph.vertices){
        string subgraph;
        string name("cluster_");
        name += (to_string(subgraph_count));
        subgraph_count++;
        subgraph += "subgraph " + name + "{\n";
        // subgraph += "label = " + name + ";\n";
        subgraph += "style = filled;\n";
        subgraph += "color = lightgrey;\n";
        subgraph += "node [style=filled,color=white];\n";
        for(auto it2 : it.aliases){
            string node_s;
            string node_name;
            node_name += "R" + std::to_string(it2.route) + "N" + std::to_string(it2.node);
            node_s +=  node_name+ "[\n";
            node_s += "pos = \"" + std::to_string(it2.pos.x / max_x) + "," + std::to_string(it2.pos.y / max_y) + "!\"\n";
            node_s += "label = " + node_name + "\n";
            node_s += "color = ";
            node_s += graphviz_color[it2.route];
            node_s += "\n";
            subgraph += node_s + "]\n";
        }
        if(dist(it.aliases.begin()->pos, pos) < CLOSE_ENOUGH){
            subgraph += "Tircgo[pos = \"" + to_string(coor.x / max_x) + "," + to_string(coor.y / max_y) + "!\"\n";
            subgraph += "label = Tircgo\n";
            subgraph += "color = khaki4\n";
            subgraph += "]\n";
        }
        subgraph += "}\n";
        digraph += subgraph;
    }
    for(int i = 0; i < this->rn_img.size(); i++){
        for(int j = 1; j < this->rn_img[i].size(); j++){
            digraph += "R" + to_string(i) + "N" + to_string(j - 1) +
                " -- " + "R" + to_string(i) + "N" + to_string(j) + " ";
            for(auto it : ((this->graph).edges)[this->rn_img[i][j - 1]]){
                if(it.dst == this->rn_img[i][j]){
                    digraph += "[label = " + to_string(it.w) + "]\n";
                    break;
                }
            }
        }
        digraph += "\n";
    }
    digraph += "}";
    return digraph;
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
        case Opcode::OPCODE_SHUTDOWN:
            this->shutdown();
            break;
        default:
            ret = false;
            break;
    }
    return ret;
}