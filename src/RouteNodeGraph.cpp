#include "RouteNodeGraph.h"
// 2D-vector version
list<geometry_msgs::Quaternion> flip_path(list<geometry_msgs::Quaternion>& _path)
{
    auto flipped = _path;
    flipped.reverse();
    for(auto& it:flipped){
        it.x = -it.x;
        it.y = -it.y;
        it.w = -it.w;
    }
    return flipped;
}
double calc_dist(list<geometry_msgs::Quaternion>& _path)
{
    double ret = 0;
    for(auto it : _path)
        ret += sqrt(it.x * it.x + it.y * it.y);
    return ret;
}
// template specialization on tircgo_msgs::RouteNode
template<>
void Graph<RouteNode>::add_node(const RouteNode& _added, double _w)
{
    // receive route >= 0 case
    if(_added.route >= 0){
        // put routes in until size match, if naming not consecutive, there will be some empty routes
        while(this->nodes.size() <= _added.route){
            this->nodes.push_back(vector<RouteNode>());
            this->weights.push_back(vector<double>());
        }
        // node naming must be consecutive else reject.
        if(this->nodes[_added.route].size() + 1 != _added.node){
            ROS_ERROR("R:%d, N:%d, W:%.1f @ (%.1f, %.1f) node addition rejected, since naming not consecutive",
                _added.route, _added.node, _w, _added.pos.x, _added.pos.y);
        }
        else{
            this->nodes[_added.route].push_back(_added);
            this->weights[_added.route].push_back(_w);
            ROS_INFO("R:%d, N:%d, W:%.1f @ (%.1f, %.1f) node added",
                _added.route, _added.node, _w, _added.pos.x, _added.pos.y);
        }
    }
    else{
        ROS_ERROR("Route <0 Error, node addition rejected");
    }
}
/* assume every route share [0] */
template<>
double Graph<RouteNode>::cost_to_target(const RouteNode& _curr, const RouteNode& _target)
{
    double ret = 0;
    for(int i = _curr.node; i > 0; i--){
        ret += this->weights[_curr.route][i];
    }
    for(int i = 1; i <= _target.node; i++){
        ret += this->weights[_target.route][i];
    }
    return ret;
}
/* assume _curr -> [any][0] -> _target */
template<>
list<RouteNode> Graph<RouteNode>::path_to_target(const RouteNode& _curr, const RouteNode& _target)
{
    list<RouteNode> ret;
    if(_curr.route != _target.route){
        for(int i = _curr.node; i >= 0; i--){
            ret.push_back(this->nodes[_curr.route][i]);
        }
        for(int i = 0; i <= _target.node; i++){
            ret.push_back(this->nodes[_target.route][i]);
        }
    }
    else{
        for(int i = _curr.node; i != _target.node; i = (_target.node > _curr.node) ? i + 1 : i - 1){
            ret.push_back(this->nodes[_curr.route][i]);
        }
    }
    return ret;
}
template<>
void Graph<RouteNode>::clear()
{
    this->nodes.clear();
    this->weights.clear();
    ROS_INFO("graph cleared");
}