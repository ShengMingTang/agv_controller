#ifndef ROUTENODEGRAPH_H
#define ROUTENODEGRAPH_H
#include <ros/ros.h>
#include <vector>
#include <list>
#include <geometry_msgs/Pose.h>
#include "pybot.h"
#include "agv_controller/RouteNode.h"
using namespace std;
using namespace pybot;

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
        void add_node(const T& _added, list<geometry_msgs::Quaternion>& _path);
        double cost_to_target(const T& _curr, const T& _target);
        list<T> path_to_target(const T& _curr, const T& _target);
        void clear();
    private:
        vector< vector<T> > nodes;
        vector< vector<double> > weights;
    };
}
#endif