#include "Controller.h"
bool Controller::nodeocp_serve(WifiNodeOcp::Request &_req, WifiNodeOcp::Response &_res)
{
    _res.is_ocp = !( (this->ocp_vptr->aliases.find(_req.q_rn)) == this->ocp_vptr->aliases.end() );
    _res.error_code = WIFI_ERRCODE_NONE;
    return true;
}
bool Controller::nodecost_serve(WifiNodeCost::Request &_req, WifiNodeCost::Response &_res)
{
    if(this->is_target_valid(_req.target.route, _req.target.node)){
        _res.cost = this->graph.shortest_path(this->ocp_vptr, this->rn_img[_req.target.route][_req.target.node]).first;
        _res.error_code = WIFI_ERRCODE_NONE;
    }
    else{
        _res.cost = numeric_limits<double>::max();
        _res.error_code = WIFI_ERRCODE_COST;
        ROS_WARN("[NodeCost Service] target out of range, reply max double value");
    }
    return true;
}
bool Controller::task_confirm_serve(WifiTaskConfirm::Request &_req, WifiTaskConfirm::Response &_res)
{
    if(this->stage_bm & MODE_AUTO){
        _res.is_taken = false;
        _res.error_code = WIFI_ERRCODE_ROBOT;
        ROS_WARN("[Task confirm Service] In auto mode, not taking any wifi allocated jobs");
    }
    else if(this->is_target_valid(_req.task.target.route, _req.task.target.node)){
        _res.is_taken = true;
        auto path = this->graph.shortest_path(this->ocp_vptr, this->rn_img[_req.task.target.route][_req.task.target.node]).second;
        this->work_list.insert(this->work_list.end(), path.begin(), path.end());
        _res.error_code = WIFI_ERRCODE_NONE;
    }
    else{
        _res.is_taken = false;
        _res.error_code = WIFI_ERRCODE_NODE;
        ROS_WARN("[Task confirm Service] target out of range, reply max double value");
    }
    return true;
}
bool Controller::askdata_serve(Ask_Data::Request &_req, Ask_Data::Response &_res)
{
    CtrlData data;
    data.mode = (int16_t)this->mode;
    data.stage_bm = this->stage_bm;

    data.tracking_status = (int16_t)this->tracking_status;
    data.lidar_levels = this->lidar_levels;
    
    geometry_msgs::Quaternion p = this->pose_tracer.get_coor();
    data.coor.x = p.x, data.coor.y = p.y, data.coor.z = p.z;
    
    data.nd_training = this->nd_training;
    data.graph = this->dumps_graph();

    data.nd_ocp = *(this->ocp_vptr->aliases.begin());
    data.nd_target = this->nd_target;
    for(auto it : this->work_list){
        data.work_list.push_back(*(it->aliases.begin()));
    }
    _res.ctrlData = data;
    return true;
}
bool Controller::is_target_ocp(const VertexType *vptr)
{
    tircgo_msgs::WifiNodeOcp srv;
    srv.request.q_rn = *(vptr->aliases.begin());
    if(this->control & CONTROL_WIFI){
        if(!this->nodeocp_clt.call(srv)){
            ROS_ERROR("| ----> Wifi Err");
            return false;
        }
        return srv.response.error_code == WIFI_ERRCODE_NONE && srv.response.is_ocp;
    }
    else{
        return false;
    }
}
void Controller::execute_schedule(const tircgo_controller::scheduleGoalConstPtr &_goal)
{
    ros::Rate r(1);
    bool success = true;

    sch_feedback.feedback.clear();
    sch_feedback.args.clear();
    
    if(!(this->stage_bm & MODE_AUTO)){
        this->sch_srv.setAborted(this->sch_res);
        r.sleep();
        return;
    }

    ROS_INFO("Action : %s taken", _goal->act.c_str());
    for(auto it : _goal->args){
        ROS_INFO("Args : %d", it);
    }
    if(_goal->act == "go"){
        /* just push node here*/
        if(this->stage_bm & MODE_TRAINING){
            int route = _goal->args[0], node = _goal->args[1];
            if(this->is_target_valid(route, node)){
                this->work_list.clear();
                auto path = this->graph.shortest_path(this->ocp_vptr, this->rn_img[route][node]).second;
                this->work_list.insert(this->work_list.end(), path.begin(), path.end());
                stringstream ss;
                for(auto it : this->work_list){
                    RouteNode nd = *(it->aliases.begin());
                    ss << "((" << nd.route << ", " << nd.node << "))->";
                }
                this->sch_feedback.feedback = ss.str();
                this->sch_srv.publishFeedback(this->sch_feedback);
            }
            else{
                this->sch_feedback.feedback = "Target out of range, plz reset";
                this->sch_srv.publishFeedback(this->sch_feedback);
                success = false;
            }
        }

        /* check finished for whole nodes path*/
        while(!this->work_list.empty()){
            VertexType *next_vptr = this->work_list.front();
            
            // trigger working
            if(!(this->is_target_ocp(next_vptr))){
                RouteNode next_nd = *(next_vptr->aliases.begin());
                vector<int16_t> work_args = {next_nd.route, next_nd.node};
                auto srv = this->base_driver.invoke(OPCODE_WORK_BEGIN, work_args);
                if(this->base_driver.is_invoke_valid(srv)){
                    this->ocp_vptr = this->work_list.front(); // claim
                    ROS_INFO("Claim the target node is occupied by the current robot");
                    ROS_INFO("Enter working mode");
                }
            }
            else{
                ROS_INFO("Want to work but target is occupied, waiting");
            }

            // Polling finished for the current small target
            while(!this->working()){
                if(this->sch_srv.isPreemptRequested() || !ros::ok()){
                    ROS_INFO("Auto mode preempted");
                    this->sch_srv.setPreempted();
                    success = false;
                    break;
                }
                r.sleep();
            }
        }
    }
    else if(_goal->act ==  "delay"){
        ros::Time start_time = ros::Time::now();
        if(_goal->args.size() > 0){
            while((ros::Time::now() - start_time).toSec() * 1000 < _goal->args[0]){
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
    }
    if(success){
        this->sch_res.res = "done";
        ROS_INFO("Action : %s done\n", _goal->act.c_str());
        this->sch_srv.setSucceeded(this->sch_res);
    }
}