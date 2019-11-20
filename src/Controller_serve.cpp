#include "Controller.h"
bool Controller::nodeocp_serve(WifiNodeOcp::Request &_req, WifiNodeOcp::Response &_res)
{
    _res.is_ocp = !( (this->ocp_vptr->aliases.find(_req.q_rn)) == this->ocp_vptr->aliases.end() );
    _res.error_code = WIFI_ERR_NONE;
    return true;
}
bool Controller::nodecost_serve(WifiNodeCost::Request &_req, WifiNodeCost::Response &_res)
{
    VertexType *v_start, *v_end;
    _res.cost = this->graph.shortest_path(this->ocp_vptr, this->target_vptr).first;
    _res.error_code = WIFI_ERR_NONE;
    return true;
}
bool Controller::task_confirm_serve(WifiTaskConfirm::Request &_req, WifiTaskConfirm::Response &_res)
{
    _res.is_taken = true;
    auto path = this->graph.shortest_path(this->ocp_vptr, this->target_vptr).second;
    this->work_list.insert(this->work_list.end(), path.begin(), path.end());
    _res.error_code = WIFI_ERR_NONE;
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
    data.nd_target = *(this->target_vptr->aliases.begin());
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
        return srv.response.error_code == WIFI_ERR_NONE && srv.response.is_ocp;
    }
    else{
        return false;
    }
}