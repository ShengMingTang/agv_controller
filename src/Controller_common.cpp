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

,sch_srv(this->n, ROBOT_SCHEDULER_CONTROLLER, boost::bind(&Controller::execute_schedule, this, _1), false)
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
            ROS_INFO("| [-f]: int = 20, operating frequency           |");
            ROS_INFO("| [-d]: float = 0.4, drive timeout              |");
            ROS_INFO("| [-p]: int = 60, precision for merging nodes   |");
            ROS_INFO("| [-m]: int = 0, set control (using bitmap      |");
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
        else{
            ROS_INFO("Unknown option, neglected");
        }
    }
    if(this->ok()){
        ROS_INFO("============================================================");
        ROS_INFO("Controller Setup :");
        ROS_INFO("> Only objects in front/back could block, neglect side objects");
        ROS_INFO("> System parameter :");
        // ROS_INFO("> Loop at frequency %f", this->loop_rate.cycleTime().toSec());
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
        // decode mode dependent instructions
        switch(this->mode){
            case MODE_IDLE:
                if(buttons[JOYBUTTON_Y])
                    ret = OPCODE_CALIB;
                else if(buttons[JOYBUTTON_X])
                    ret = OPCODE_TRAIN_BEGIN;
                else if(buttons[JOYBUTTON_B])
                    ret = OPCODE_WORK_BEGIN;

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

    // trained but not in training
    if((this->stage_bm & MODE_TRAINING) && (this->mode != MODE_TRAINING || this->nd_training.node == -1)){
        _vel = {0, 0};
    }
    this->op_vel = _vel;

    // legal condition to drive
    double t = (ros::Time::now() - this->pose_tracer.get_starttime()).toSec();
    if(t >= this->drive_timeout || _vel[0] != this->pose_tracer.get_vel()[0] || _vel[1] != this->pose_tracer.get_vel()[1]){ // different motion
        if(_vel[0] == 0 && this->pose_tracer.get_vel()[0] == 0 &&
           _vel[1] == 0 && this->pose_tracer.get_vel()[1] == 0){
               // don't send redundant 0 vel
        }
        else{
            if(this->check_safety()){
                auto srv = this->base_driver.invoke(OPCODE_DRIVE, {_vel[0], _vel[1]});
                if(this->base_driver.is_invoke_valid(srv)){
                    this->pose_tracer.set_vw(_vel);
                }
            }
            else{
                ROS_INFO("Unsafe condition, drive rejected");
                return false;
            }
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
    ROS_INFO("Clear data");
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
/* subsriber to track status */
void Controller::status_tracking(const RobotStatus::ConstPtr& _msg)
{
    if(_msg->is_activated){
        this->mode = _msg->now_mode;
        this->stage_bm |= _msg->now_mode;
        switch (this->mode)
        {
            case MODE_TRAINING:
                this->base_driver.invoke(OPCODE_SIGNAL, {DEVICE_LED_G, DEVICE_LED_OFF, 1});
                this->base_driver.invoke(OPCODE_SIGNAL, {DEVICE_LED_Y, DEVICE_LED_ON, 1});
                break;
            case MODE_WORKING:
                this->base_driver.invoke(OPCODE_SIGNAL, {DEVICE_LED_Y, DEVICE_LED_OFF, 1});
                this->base_driver.invoke(OPCODE_SIGNAL, {DEVICE_LED_G, DEVICE_LED_ON, 1});
            default:
                break;
        }
        if(_msg->tracking_status_reply.is_activated){
            this->tracking_status = _msg->tracking_status_reply.reply;
            switch (this->tracking_status)
            {
                case TRACKING_STATUS_NONE:
                case TRACKING_STATUS_NORMAL:
                    break;
                case TRACKING_STATUS_ARRIVAL:
                    this->base_driver.invoke(OPCODE_SIGNAL, {DEVICE_LED_Y, DEVICE_LED_ON, 1});
                    ROS_INFO("Arrival on target");
                    break;
                default:
                    this->base_driver.invoke(OPCODE_SIGNAL, {DEVICE_LED_Y, DEVICE_LED_OFF, 1});
                    break;
            }
        }
        else if(this->mode & MODE_WORKING){
            ROS_WARN("Tracking_status_reply not activated");
        }
        if(_msg->lidar_level_reply.is_activated){
            this->lidar_levels = _msg->lidar_level_reply.level_reply;
        }
        else{
            ROS_WARN("Lidar_level_reply not activated");
        }
    }
    else{
        ROS_ERROR("RobotStatus not activated");
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
    sprintf(buff, "mode:%d, %d, trk:%d, pos:(%+.1f,%+.1f %+.1f), v:<%+2d,%+2d>, L:[%d,%d,%d,%d], Trn{%d, %d}, Tar<%d,%d>",
                this->mode, this->stage_bm,
                this->tracking_status,
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
    const char graphviz_color[5][20] = {"indianred", "orange3", "yellow3", "green3", "lightblue3"};
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
        if(dist(it.aliases.begin()->pos, pos) < this->close_enough){
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
            // this->sch_srv.shutdown();
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