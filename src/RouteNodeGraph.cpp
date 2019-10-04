#include "RouteNodeGraph.h"
// template specialization
template<>
void Graph<wifi::RouteNode>::add_node(wifi::RouteNode _added, Edge<wifi::RouteNode> _edge)
{

    if(_added.route >= 0){
        while(this->nodes.size() <= _added.route){
            this->nodes.push_back(vector<wifi::RouteNode>());
            this->weights.push_back(vector<double>());
        }
        this->nodes[_added.route].push_back(_added);
        this->weights[_added.route].push_back(_edge.w);
        ROS_INFO("R:%d, N:%d, W:%.1f @ (%.1f, %.1f) added", _added.route, _added.node, _edge.w, _added.pos.x, _added.pos.y);
    }
    else{
        ROS_ERROR("RouteNode Wifi Pass Error");
    }
}
template<>
double Graph<wifi::RouteNode>::cost_to_target(const wifi::RouteNode& _curr, const wifi::RouteNode& _target)
{
    double ret = 0;
    // assume _curr -> origin -> _target
    for(int i = _curr.node; i > 0; i--){
        ret += this->weights[_curr.route][i];
    }
    for(int i = 0; i <= _target.node; i++){
        ret += this->weights[_target.route][i];
    }
    return ret;
}
template<>
list<wifi::RouteNode> Graph<wifi::RouteNode>::path_to_target(const wifi::RouteNode& _curr, const wifi::RouteNode& _target)
{
    list<wifi::RouteNode> ret;
    // assume _curr -> origin -> _target
    for(int i = _curr.node; i > 0; i--){
        ret.push_back(this->nodes[_curr.route][i]);
    }
    for(int i = 0; i <= _target.node; i++){
        ret.push_back(this->nodes[_target.route][i]);
    }
    return ret;
}
template<>
void Graph<wifi::RouteNode>::clear()
{
    this->nodes.clear();
    this->weights.clear();
}