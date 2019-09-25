#ifndef TIRCGO_COMMON_H
#define TIRCGO_COMMON_H
/* tircgo specialized data structures */
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include "tircgo_msgs/RouteNode.h"

#include <cmath>
#include <list>
#include <set>

using namespace std;
using namespace tircgo_msgs;

struct WalkUnit
{
    vector<int16_t> vel; // encoded as {linear, angular}
    ros::Duration dur; // time keep executing this walk
};

/* V is the Vertex type, W is the walkUnit type */
template<typename V, typename W>
struct Edge
{
    V *src, *dst;
    double w;
    list<W> walk;
};

/* T is the primitive type and should implement less<T> */
template<typename T>
struct Vertex
{
    geometry_msgs::Point pos;
    set< T,less<T> > aliases;
};

using WalkUnitType = WalkUnit;

list<geometry_msgs::Quaternion> flip_walk(const list<geometry_msgs::Quaternion> &_walk);
list<WalkUnit> flip_walk(const list<WalkUnit> &_walk);
bool operator==(const RouteNode &a, const RouteNode &b);
bool operator<(const RouteNode &a, const RouteNode &b);
double dist(const RouteNode &_a, const RouteNode &_b);
double dist(const geometry_msgs::Point &_a, const geometry_msgs::Point &_b);

namespace std
{
    template<> struct less<RouteNode>
    {
        bool operator() (const RouteNode& lhs, const RouteNode& rhs) const
        {
            if(lhs.route == rhs.route)
                return lhs.node < rhs.node;
            else
                return lhs.route < rhs.route;
        }    
    };
}
#endif