#include "Controller.h"
Controller::Controller(const string& _id):
frame_id{_id}
,base_driver{UART(_id)}
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
    time_t now = time(nullptr);
    this->timestamp = asctime(localtime(&now));
    this->nd_training.route = this->nd_training.node = 0;
    this->ocp_vptr = nullptr;
    this->log_path = "tircgo_log" + this->frame_id;
    // ROS_INFO("Controller constructed");
}
Controller::~Controller()
{
    // ROS_INFO("Controller destructed");
}
void Controller::setup(int _argc, char **argv)
{
    for(int i = 1; i < _argc; i++){
        string opt;
        opt = argv[i];
        if(opt == "-h"){
            ROS_INFO("|---------------------------------------------------|");
            ROS_INFO("| [-h]: display this msg                            |");
            ROS_INFO("| [-i]: string, identifier of this robot            |");
            ROS_INFO("| [-f]: int = 5, operating frequency                |");
            ROS_INFO("| [-d]: float = 0.4, drive timeout                  |");
            ROS_INFO("| [-p]: int = 60, precision for merging nodes       |");
            ROS_INFO("| [-m]: int = 0, set control (using bitmap)         |");
            ROS_INFO("| [-l]: str = tircgo_log, search for log files      |");
            ROS_INFO("| control = {wifi(0/1)}                             |");
            ROS_INFO("|---------------------------------------------------|");
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
        else if(opt == "-l"){
            if(i + 1 < _argc){
                opt = argv[i+1];
                i++;
                this->log_path = opt;
                fstream fs;
                fs.open(this->log_path, fstream::in);
                if(fs.is_open()){
                    string ans;
                    ROS_WARN("Want to restore ? [y/n]");
                    cin >> ans;
                    if(ans == "y"){
                        this->restore(fs);
                    }
                    else{
                        ROS_WARN("Skip restoration");
                    }

                }
                else{
                    ROS_ERROR("Log file sepecified but not exist, ignored");
                }
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
        ROS_INFO("> ID = %s", this->frame_id.c_str());
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
int16_t Controller::decode_opcode(sensor_msgs::Joy::ConstPtr _ptr)
{
    int16_t ret = OPCODE_NONE;
    if(!_ptr)
        return OPCODE_NONE;
    vector<float> axes = _ptr->axes;
    vector<int32_t> buttons = _ptr->buttons;
    // priviledged, exe in place
    if(buttons[JOYBUTTON_BACK]){
        this->shutdown();
        return OPCODE_SHUTDOWN;
    }
    else if(buttons[JOYBUTTON_START]){ // switch auto input
        if( !(this->stage_bm & MODE_AUTO) && (this->stage_bm & MODE_TRAINING) ){ // enter auto
            this->stage_bm |= MODE_AUTO;
            this->sch_srv.start();
            ROS_WARN("Start auto mode");
        }
        else if(this->stage_bm & MODE_AUTO){ // leave auto
            this->stage_bm &= (~MODE_AUTO);
            ROS_WARN("Finish auto mode, script has no effect now");
        }
        else{
            ROS_ERROR("Switch auto mode Error");
        }
        return OPCODE_AUTO;
    }
    else if(buttons[JOYBUTTON_RB]){ // switch agent action input
        if(this->stage_bm & MODE_AGENT){ // enter agent
            this->stage_bm &= (~MODE_AGENT);
            ROS_INFO("Turn off agent input");
        }
        else{ // leave
            this->stage_bm |= MODE_AGENT;
            this->sch_srv.start();
            ROS_INFO("Turn on agent input");
        }
        return OPCODE_AGENT;
    }

    // minor, exe in place
    if(buttons[JOYBUTTON_LB]){ // display UART history
        ROS_INFO("%s", this->base_driver.get_cmds().c_str());
    }
    if(buttons[JOYBUTTON_LT]){ // clear UART history
        this->base_driver.clear();
        ROS_INFO("Clear UART history");
    }
    if(buttons[JOYBUTTON_RT]){ // publish our UART histroy anonymously, all receiver
        tircgo_msgs::ControllerTalk msg;
        msg.author = this->timestamp;
        msg.talk = this->base_driver.get_cmds();
        this->pub_agent_imm.publish(msg);
        ROS_INFO("Publish UART history");
    }
    if(axes[JOYAXES_CROSS_LR] == -1.0)
        this->nd_target.route = (this->nd_target.route + 1) % TRAIN_ROUTE_MAX;
    if(axes[JOYAXES_CROSS_LR] == 1 && this->nd_target.route > 0){
        this->nd_target.route--;
    }
    if(axes[JOYAXES_CROSS_UD] == 1.0)
        this->nd_target.node++;
    if(axes[JOYAXES_CROSS_UD] == -1 && this->nd_target.node > 0){
        this->nd_target.node--;
    }
    if(buttons[JOYBUTTON_STICK_RIGHT] && this->nd_training.route + 1 < TRAIN_ROUTE_MAX){
        this->nd_training.route++;
    }
    if(buttons[JOYBUTTON_STICK_LEFT] && this->nd_training.route > 0){
        this->nd_training.route--;
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
    return ret;
}

/*
    decode joystick to drive signal
    independent of int16_t decoding
*/
vector<int16_t> Controller::decode_drive(sensor_msgs::Joy::ConstPtr _ptr)
{
    if(_ptr){
        vector<float> axes = _ptr->axes;
        vector<int32_t> buttons = _ptr->buttons;
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

    vector<int16_t> curr_vel = this->pose_tracer.get_vel();
    if(_vel[0] == 0 && _vel[1] == 0 && curr_vel[0] == 0 && curr_vel[1] == 0)
        return true;
    if(this->check_safety(_vel)){
        auto srv = this->base_driver.invoke(OPCODE_DRIVE, _vel);
        if(this->base_driver.is_invoke_valid(srv)){
            this->pose_tracer.set_vw(_vel);
        }
        else{
            ROS_INFO("Drive failed");
            return false;
        }
    }
    else{
        ROS_INFO("Not safe, drive neglected");
    }
    return true;
}

/* Sys related */
void Controller::clear()
{
    this->stage_bm = 0;
    this->pose_tracer.clear();
    this->graph.clear();
    for(auto &it : this->rn_img){
        it.clear();
    }
    this->work_list.clear();
    ROS_WARN("Clear data except for UART histroy (press LT to clear it if you want)");
}

/* writing log files */
void Controller::log()
{
    fstream fs;
    fs.open(this->log_path, fstream::out);
    map<VertexType*, int> v_map;

    // timestamp
    time_t result = std::time(nullptr);
    fs << std::asctime(std::localtime(&result)) << "\n";
    
    // status
    fs << this->mode << "\n";
    fs << this->stage_bm << "\n";
    fs << "\n";
    
    // vertices bookeeping
    fs << this->graph.vertices.size() << "\n";
    int count = 0;
    for(auto &it : this->graph.vertices){
        v_map[&it] = count++;
        fs << it.aliases.size() << "\n";
        fs << it.pos.x << " " << it.pos.y << " " << it.pos.z << " ";
        for(auto &it2 : it.aliases){
            fs << it2.route << " " << it2.node << " ";
        }
        fs << "\n";
    }
    fs << "\n";

    // edges bookeeping
    count = 0;
    for(auto &it : this->graph.edges){
        count += it.second.size();
    }
    fs << count << "\n";
    for(auto &it : this->graph.edges){
        for(auto &it2 : it.second){
            fs << v_map[it2.src] << " " << v_map[it2.dst] << " " << it2.w << "\n";
            fs << it2.walk.size() << " ";
            for(auto walk : it2.walk){
                fs << walk.vel[0] << " " << walk.vel[1] << " " << walk.dur.toSec() << "\n";
            }
            fs << "\n";
        }
        fs << "\n";
    }
    fs << "\n";

    // trained route book keeping
    for(int i = 0; i < TRAIN_ROUTE_MAX; i++){
        fs << this->rn_img[i].size() << "\n";
        for(int j = 0; j < this->rn_img[i].size(); j++){
            fs << v_map[this->rn_img[i][j]] << " ";
        }
        fs << "\n";
    }
    fs.close();

    ROS_INFO("Log file has been written to \"%s\"", this->log_path.c_str());
}
void Controller::restore(fstream &fs)
{
    map<int, VertexType*> v_map;
    int vertices_size, edges_size;
    
    // timestamp
    char timestamp[256];
    fs.getline(timestamp, 256);
    ROS_INFO("Last Login : %s", timestamp);

    // status
    fs >> this->mode >> this->stage_bm;

    // vertices
    fs >> vertices_size;
    for(int i = 0; i < vertices_size; i++){
        int aliases_size;
        VertexType tmp;
        v_map[i] = this->graph.add_vertex(tmp);
        fs >> aliases_size;
        fs >> v_map[i]->pos.x >> v_map[i]->pos.y >> v_map[i]->pos.z;
        
        for(int j = 0; j < aliases_size; j++){
            RouteNode nd;
            fs >> nd.route >> nd.node;
            v_map[i]->aliases.insert(nd);
        }
    }

    // edges
    fs >> edges_size;
    for(int i = 0; i < edges_size; i++){
        int src, dst;
        int walk_size;
        EdgeType e;
        fs >> src >> dst >> e.w;
        e.src = v_map[src], e.dst = v_map[dst];
        fs >> walk_size;
        for(int j = 0; j < walk_size; j++){
            WalkUnitType walk;
            int16_t linear, angular;
            float dur;
            fs >> linear >> angular >> dur;
            walk.vel.push_back(linear);
            walk.vel.push_back(angular);
            walk.dur = ros::Duration(dur);
            e.walk.push_back(walk);
        }
        this->graph.add_edge(e);
    }

    // trained route
    for(int i = 0; i < TRAIN_ROUTE_MAX; i++){
        int trained_nodes_size;
        fs >> trained_nodes_size;
        for(int j = 0; j < trained_nodes_size; j++){
            int node;
            fs >> node;
            this->rn_img[i].push_back(v_map[node]);
        }
    }
}
/* shutdown routine */
bool Controller::shutdown()
{
    this->log();
    this->stage_bm |= MODE_NOTOK;
    ROS_INFO("Controller Software Shutdown only");
    ROS_INFO("Please turn off the power manually");
}
/* safety issue */
bool Controller::check_safety(vector<int16_t> _vel)
{
    bool ret;
    if(this->mode != MODE_WORKING){
        if(_vel[0] > 0)
            return this->lidar_levels[LIDAR_DIR_FRONT] < LIDAR_LEVEL_CLOSE;
        else if(_vel[0] < 0)
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

bool Controller::priviledged_instr(int16_t _op)
{
    switch(_op){
        // return statements
        case OPCODE_SHUTDOWN:
        case OPCODE_AUTO:
        case OPCODE_AGENT:
            return true;
        default:
            return false;
    }
}
/* return if the _route, _node pair is valid in the current robot */
bool Controller::is_target_valid(const RouteNode &_nd)
{
    if(_nd.route >= 0 && _nd.route < this->rn_img.size()){
        return _nd.node < this->rn_img[_nd.route].size(); // short check, assume nodes come in successive order
    }
    return false;
}