#include "Controller.h"
bool Controller::wifi_nodeocp_serve(WifiNodeOcp::Request &req, WifiNodeOcp::Response &res)
{
    RouteNode q_node = req.q_node;
    res.is_ocp = (this->node_ocp.route == req.q_node.route && this->node_ocp.node == req.q_node.node);
    return true;
}