#ifndef ROUTENODEGRAPH_H
#define ROUTENODEGRAPH_H
#include <ros/ros.h>
#include <vector>
#include <list>
#include "pybot.h"
#include "wifi/RouteNode.h"
using namespace std;
using namespace pybot;
using namespace wifi;
namespace pybot
{
    // typedef struct RouteNode_Edge
    // {
    //     double w; // weight
    //     wifi::RouteNodeRouteNode v; // destination
    // }Node;
    class RouteNodeGraph
    {
    public:
        RouteNodeGraph();
        ~RouteNodeGraph();
        void add_node(RouteNode _added);
        double cost_to_target(const RouteNode& _curr, const RouteNode& _target);
        list<RouteNode> path_to_target(const RouteNode& _target);
    private:
        vector< vector<RouteNode> > data;
    };
}
#endif