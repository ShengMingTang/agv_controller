#include "Controller.h"

/* subsriber to track status */
void Controller::status_tracking(const RobotStatus::ConstPtr& _msg)
{
    if(_msg->is_activated){
        this->mode = _msg->now_mode;
        this->stage_bm |= _msg->now_mode;
        if(_msg->tracking_status_reply.is_activated){
            this->tracking_status = _msg->tracking_status_reply.reply;
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
/* echo status */
void Controller::monitor_display() const
{
    stringstream ss;
    char buff[150];
    auto coor = this->pose_tracer.get_coor();
    RouteNode nd;
    
    if(this->ocp_vptr){
        nd = *(this->ocp_vptr->aliases.begin());
    }
    else{
        nd.route = nd.node = -1;
    }

    snprintf(buff, 150, "stat:%d,%d, trk:%d, ocp:%d,%d, pos:(%+.1f,%+.1f %+.1f), v:<%+2d,%+2d>, L:[%d,%d,%d,%d], Trn{%d, %d}, Tar<%d,%d>",
                this->mode, this->stage_bm,
                this->tracking_status,
                nd.route, nd.node,
                coor.x, coor.y, (coor.w) / 3.14159 * 180,
                this->pose_tracer.get_vel()[0], this->pose_tracer.get_vel()[1],
                this->lidar_levels[0], this->lidar_levels[1], this->lidar_levels[2], this->lidar_levels[3],
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
    if(this->graph.is_up_to_date){
        return this->graph_viz;
    }

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
    this->graph_viz = digraph;
    return digraph;
}
/* wifi */
bool Controller::nodeocp_serve(WifiNodeOcp::Request &_req, WifiNodeOcp::Response &_res)
{
    if(this->ocp_vptr){
        _res.is_ocp = !( this->ocp_vptr->aliases.find(_req.q_rn) == this->ocp_vptr->aliases.end() );
        _res.error_code = WIFI_ERRCODE_NONE;
        ROS_INFO("[Nodeocp Service] Got asked if on R%dN%d, reply:%d", _req.q_rn.route, _req.q_rn.node, _res.is_ocp);
    }
    else{
        #ifdef ROBOT_CONTROLLER_TEST
            string ans;
            ROS_WARN("{Nodeocp Service] manually answer if occupy R%dN%d ? [y/n]", _req.q_rn.route, _req.q_rn.node);
            cin >> ans;
            _res.is_ocp = (ans == "y");
            _res.error_code = WIFI_ERRCODE_NONE;
        #else
            ROS_WARN("[Nodeocp Service] target out of range, reply false (not occupied)");
            _res.is_ocp = false;
            _res.error_code = WIFI_ERRCODE_NODE;
        #endif
    }
    
    return true;
}
bool Controller::nodecost_serve(WifiNodeCost::Request &_req, WifiNodeCost::Response &_res)
{
    if(this->is_target_valid(_req.target)){
        _res.cost = this->graph.shortest_path(this->ocp_vptr, this->rn_img[_req.target.route][_req.target.node]).first;
        _res.error_code = WIFI_ERRCODE_NONE;
        ROS_INFO("[Nodecost Service] Got asked cost to R%dN%d, reply:%.1f", _req.target.route, _req.target.node, _res.cost);
    }
    else{
        _res.cost = numeric_limits<double>::max();
        _res.error_code = WIFI_ERRCODE_COST;
        ROS_WARN("[Nodecost Service] target out of range, reply max double value");
    }
    return true;
}
bool Controller::task_confirm_serve(WifiTaskConfirm::Request &_req, WifiTaskConfirm::Response &_res)
{
    if(this->stage_bm & MODE_AUTO){
        _res.is_taken = false;
        _res.error_code = WIFI_ERRCODE_ROBOT;
        ROS_WARN("[Taskconfirm Service] In auto mode, not taking any wifi allocated jobs");
    }
    else if(this->is_target_valid(_req.task.target)){
        _res.is_taken = true;
        auto path = this->graph.shortest_path(this->ocp_vptr, this->rn_img[_req.task.target.route][_req.task.target.node]).second;
        this->work_list.insert(this->work_list.end(), path.begin(), path.end());
        _res.error_code = WIFI_ERRCODE_NONE;
    }
    else{
        _res.is_taken = false;
        _res.error_code = WIFI_ERRCODE_NODE;
        ROS_WARN("[Taskconfirm Service] target out of range");
    }
    return true;
}
bool Controller::askdata_serve(Ask_Data::Request &_req, Ask_Data::Response &_res)
{
    CtrlData data;
    data.mode = (int16_t)this->mode;
    data.stage_bm = this->stage_bm;

    // data.tracking_status = (int16_t)this->tracking_status;
    // data.lidar_levels = this->lidar_levels;
    
    // save io burden
    // geometry_msgs::Quaternion p = this->pose_tracer.get_coor();
    // data.coor.x = p.x, data.coor.y = p.y, data.coor.z = p.z;
    
    // data.nd_training = this->nd_training;
    // data.graph = this->dumps_graph();

    // if(this->ocp_vptr && !this->ocp_vptr->aliases.empty()){
    //     data.nd_ocp = *(this->ocp_vptr->aliases.begin());
    // }
    // else{
    //     data.nd_ocp.route = data.nd_ocp.node = -1;
    // }

    // data.nd_target = this->nd_target;
    // for(auto it : this->work_list){
    //     data.work_list.push_back(*(it->aliases.begin()));
    // }
    _res.ctrlData = data;
    return true;
}

/* true then the question node is occupied or being computed */
bool Controller::is_target_ocp(const VertexType *vptr)
{
    tircgo_msgs::WifiNodeOcp srv;
    srv.request.q_rn = *(vptr->aliases.begin());
    if(this->control & CONTROL_WIFI && vptr){
        if(!this->nodeocp_clt.call(srv)){
            ROS_ERROR("| ----> Wifi SrvErr");
            return false;
        }
        if(srv.response.error_code == WIFI_ERRCODE_NONE){
            return srv.response.is_ocp;
        }
        else{
            return true;
        }
    }
    else{
        return false;
    }
}
/* scheduler */
void Controller::execute_schedule(const tircgo_controller::scheduleGoalConstPtr &_goal)
{
    ros::Rate r(0.2);
    bool success = true;

    sch_feedback.feedback.clear();
    sch_feedback.args.clear();

    // controller itself can't distinguish teach or auto
    if((this->stage_bm & MODE_AUTO) || (this->stage_bm & MODE_AGENT)){
        ROS_INFO("Action : %c taken", _goal->act);
        stringstream ss;
        ss << "Args : ";
        for(auto it : _goal->args){
            ss << it << " ";
        }
        ROS_INFO("%s", ss.str().c_str());
        if(_goal->act == OPCODE_WORK_BEGIN){
            /* just push node here*/
            RouteNode nd;
            nd.route = _goal->args[0], nd.node = _goal->args[1];
            bool valid = this->add_target(nd);
            if(valid){
                this->sch_feedback.feedback = "take";
                this->sch_srv.publishFeedback(this->sch_feedback);
                /* check finished for whole nodes path */
                bool is_empty;
                this->work_mutex.lock();
                is_empty = this->work_list.empty();
                this->work_mutex.unlock();
                while(this->stage_bm & MODE_AUTO && !is_empty){
                    if(this->mode != MODE_WORKING){
                        this->work_begin();
                    }
                    this->working(OPCODE_NONE);
                    if(this->sch_srv.isPreemptRequested() || !ros::ok()){
                        ROS_INFO("Auto mode preempted");
                        this->sch_srv.setPreempted();
                        success = false;
                        break;
                    }
                    r.sleep();
                    this->work_mutex.lock();
                    is_empty = this->work_list.empty();
                    this->work_mutex.unlock();
                }
            }
            else{
                this->sch_feedback.feedback = "Invalid";
                this->sch_srv.publishFeedback(this->sch_feedback);
                success = false;
            }

        }
        else if(_goal->act == OPCODE_DELAY){
            this->drive({0, 0});
            ros::Time t1 = ros::Time::now();
            double t = (ros::Time::now() - t1).toSec() * 100;
            while(static_cast<int16_t>(t) < _goal->args[0]){
                // delay
                if(this->sch_srv.isPreemptRequested() || !ros::ok()){
                    ROS_INFO("Auto mode preempted");
                    this->sch_srv.setPreempted();
                    success = false;
                    break;
                }
                r.sleep();
            }
        }
        else if(_goal->act == OPCODE_DRIVE){
            ros::Time t1 = ros::Time::now();
            double t;
            do{
                this->drive({_goal->args[0], _goal->args[1]});
                t = (ros::Time::now() - t1).toSec();
                if(this->sch_srv.isPreemptRequested() || !ros::ok()){
                    ROS_INFO("Auto mode preempted");
                    this->sch_srv.setPreempted();
                    success = false;
                    break;
                }
            }while(static_cast<int16_t>(t) * 100 < _goal->args[2] && this->stage_bm & MODE_AUTO);
        }
        else if(_goal->act == OPCODE_TRAIN_BEGIN){
            success = this->train_begin();
            if(!success){
                ROS_ERROR("Train begin failed");
            }
            if(this->sch_srv.isPreemptRequested() || !ros::ok()){
                ROS_INFO("Auto mode preempted");
                this->sch_srv.setPreempted();
                success = false;
            }
        }
        else if(_goal->act == OPCODE_SETNODE){
            success = this->set_node();
            if(!success){
                ROS_ERROR("Direct SetNode failed");
            }
            if(this->sch_srv.isPreemptRequested() || !ros::ok()){
                ROS_INFO("Auto mode preempted");
                this->sch_srv.setPreempted();
                success = false;
            }
        }
        else if(_goal->act == OPCODE_TRAIN_FINISH){
            success = this->train_finish();
            if(!success){
                ROS_ERROR("Train finish failed");
            }
            if(this->sch_srv.isPreemptRequested() || !ros::ok()){
                ROS_INFO("Auto mode preempted");
                this->sch_srv.setPreempted();
                success = false;
            }
        }
        else{
            success = false;
            ROS_ERROR("Unknown Action, ignored");
        }
    }
    else{
        success = false;
    }
    if(success){
        this->sch_res.res = "done";
        ROS_INFO("Action : %c done\n", _goal->act);
        this->sch_srv.setSucceeded(this->sch_res);
    }
    else{
        this->sch_res.res = "failed";
        // ROS_INFO("Action : %c failed\n", _goal->act);
        this->sch_srv.setAborted(this->sch_res);
    }
}