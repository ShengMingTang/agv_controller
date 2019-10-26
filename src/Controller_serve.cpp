#include "Controller.h"
bool Controller::nodeocp_serve(WifiNodeOcp::Request &_req, WifiNodeOcp::Response &_res)
{
    Node nd = this->nd_ocp;
    _res.is_ocp = (_req.q_rn.route == nd.route && _req.q_rn.node == nd.node);
    _res.error_code = WIFI_ERR_NONE;
    return true;
}
bool Controller::nodecost_serve(WifiNodeCost::Request &_req, WifiNodeCost::Response &_res)
{
    _res.cost = this->graph.cost_to_target(this->nd_ocp, _req.target);
    _res.error_code = WIFI_ERR_NONE;
    return true;
}
bool Controller::task_confirm_serve(WifiTaskConfirm::Request &_req, WifiTaskConfirm::Response &_res)
{
    _res.error_code = WIFI_ERR_NONE;
    _res.is_taken = true;
    auto path = this->graph.path_to_target(this->nd_ocp, _req.task.target);
    this->work_list.insert(this->work_list.end(), path.begin(), path.end());
    return true;
}
bool Controller::askdata_serve(Ask_Data::Request &_req, Ask_Data::Response &_res)
{
    CtrlData data;
    data.mode = (int16_t)this->mode;
    data.coor = this->pose_tracer.get_coor();
    data.tracking_status = (int16_t)this->tracking_status;
    data.lidar_levels = this->lidar_levels;
    data.is_origin_set = this->is_origin_set;
    data.is_calibed = this->is_calibed;
    data.is_trained = this->is_trained;
    data.nd_training = this->nd_training;
    data.nd_target = this->nd_target;
    data.nd_ocp = this->nd_ocp;
    data.work_list = vector<Node>(this->work_list.begin(), this->work_list.end());
    _res.SMData = data;
    return true;
}