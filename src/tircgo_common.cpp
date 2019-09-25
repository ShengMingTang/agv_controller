
#include "tircgo_common.h"
list<geometry_msgs::Quaternion> flip_walk(const list<geometry_msgs::Quaternion> &_walk)
{
    list<geometry_msgs::Quaternion> flipped = _walk;
        flipped.reverse();
        for(auto &it:flipped){
            it.x = -it.x;
            it.y = -it.y;
            it.z = -it.z;
            it.w = -it.w;
        }
        return flipped;
}

list<WalkUnit> flip_walk(const list<WalkUnit> &_walk)
{
    list<WalkUnit> flipped = _walk;
        flipped.reverse();
        for(auto &it:flipped){
            it.vel[0] *= (-1);
            it.vel[1] *= (-1);
        }
        return flipped;
}
bool operator==(const RouteNode &a, const RouteNode &b)
{
    return (a.route == b.route) && (a.node == b.node);
}
bool operator<(const RouteNode &a, const RouteNode &b)
{
    if(a.route == b.route)
        return a.node < b.node;
    else
        return a.route < b.route;
}

double dist(const RouteNode &_a, const RouteNode &_b)
{
    double del_x = (_a.pos.x) - (_b.pos.x), del_y = (_a.pos.y - _b.pos.y);
    return del_x * del_x + del_y * del_y;
}
double dist(const geometry_msgs::Point &_a, const geometry_msgs::Point &_b)
{
    return sqrt( pow(_a.x - _b.x, 2) + pow(_a.y - _b.y, 2) );
}