#ifndef NODE_WRAPPER_H
#define NODE_WRAPPER_H
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <list>

#define CLOSE_THRES 1

using namespace std;

template<typename T>
double dist(list<T>& _path);

template<typename T>
double dist(const T &_a, const T &_b);

template<typename T>
struct Edge
{
    Edge(const T& _dest, list<geometry_msgs::Vector3>& _path)
    :v{_dest}
    ,w{dist(_path)}
    ,walk{_path}
    {}
    ~Edge()
    {}
    T v; // adjacent node
    double w = 0; // weight
    list<geometry_msgs::Vector3> walk; // a series of motion to v
};

template<typename T>
class Node_w
{
public:
    Node_w() {}
    Node_w(const T &_base);
    void add_edge(Edge<T> _e); // create an edge to _adj node_wrapper
    void add_alias(const T &_added);
    bool is_alias(const T &_alias)const;
    bool is_close(const T &_other)const;
private:
    geometry_msgs::Point point;
    vector< Edge<T> > edges;
    vector<T> aliases;
};

template<typename T>
class Graph
{
public:
    Graph() {}
    ~Graph() {}
    void add_node(const T& _added);
    double cost_to_target(const T& _curr, const T& _target);
    double cost_to_target(const Node_w<T>& _curr, const Node_w<T> &_target);
    list<T> path_to_target(const T& _curr, const T& _target);
    list< Node_w<T> > path_to_target(const Node_w<T>& _curr, const Node_w<T> &_target);
    void clear();
private:
    vector< Node_w<T> > nodes;
    vector< vector< Node_w<T>* > > primitives;
};
#endif