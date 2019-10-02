#ifndef ROUTENODEGRAPH_H
#define ROUTENODEGRAPH_H
#include <ros/ros.h>
#include <vector>
#include <list>
#include "pybot.h"
#include "wifi/RouteNode.h"
#include <geometry_msgs/Pose.h>
using namespace std;
using namespace pybot;
using namespace wifi;
namespace pybot
{
    // RouteNode defined in wifi pkg
    template<typename T>
    struct Edge
    {
        T v; // this node, adjacent node
        geometry_msgs::Pose pose;
        double w; // weight
        
    };
    template<typename T>
    class Graph
    {
    public:
        Graph() {}
        ~Graph() {}
        void add_node(T _added, double _w);
        double cost_to_target(const T& _curr, const T& _target);
        list<T> path_to_target(const T& _curr, const T& _target);
    private:
        vector< vector<T> > nodes;
        vector< vector<double> > weights;
    };
}
#endif