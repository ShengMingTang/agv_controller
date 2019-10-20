#ifndef ROUTENODEGRAPH_H
#define ROUTENODEGRAPH_H
#include <ros/ros.h>
#include <vector>
#include <list>
#include <geometry_msgs/Quaternion.h>
#include "pybot.h"
#include "tircgo_msgs/RouteNode.h"
using namespace std;
using namespace pybot;
using namespace tircgo_msgs;
// Layout
// [0, 0] - [0, 1] - [0, 2] - ...
//   ||
// [1, 0] - [1, 1] - [1, 2] - ...
//   ||
// [2, 0] - ...
//   ||
//   .
//   .
//   .
namespace pybot
{
    list<geometry_msgs::Quaternion> flip_path(list<geometry_msgs::Quaternion>& _path);
    template<typename T>
    struct Edge
    {
        Edge(const T& _dest, list<geometry_msgs::Quaternion>& _path)
        :v{_dest}
        {
            for(auto it : _path)
                w += sqrt(it.x * it.x + it.y * it.y);
        }
        ~Edge() {}
        T v; // adjacent node
        list<geometry_msgs::Quaternion> walk; // a series of motion to v
        double w = 0; // weight
    };
    template<typename T>
    class Graph
    {
    public:
        Graph() {}
        ~Graph() {}
        void add_node(const T& _added, double _w);
        double cost_to_target(const T& _curr, const T& _target);
        list<T> path_to_target(const T& _curr, const T& _target);
        void clear();
    private:
        vector< vector<T> > nodes;
        vector< vector<double> > weights; // weights[i][j] maps the cost from nodes[i][j-1] to nodes[i][j]
    };
}
#endif