#include "PoseTracer.h"
#include <cmath>
#define PI 3.14159
PoseTracer::PoseTracer():
starttime{ros::Time::now()}
{
    ROS_INFO("PoseTracer constructed");
}
PoseTracer::~PoseTracer()
{
    ROS_INFO("PoseTracer destructed");
}
double PoseTracer::roundPi(double _w)const
{
    while(_w < 0)
        _w += 2 * PI;
    while(_w >= 2 * PI)
        _w -= 2 * PI;
    return _w;
}
geometry_msgs::Quaternion PoseTracer::get_coor()const
{
    geometry_msgs::Quaternion tmp;
    tmp.x = this->coor.x + this->vel.x * (ros::Time::now() - this->starttime).toSec();
    tmp.y = this->coor.y + this->vel.y * (ros::Time::now() - this->starttime).toSec();
    tmp.w = this->coor.w + this->vel.w * (ros::Time::now() - this->starttime).toSec();
    tmp.w = this->roundPi(tmp.w);
    return tmp;
}
double PoseTracer::get_dist()
{
    double ret = 0;
    for(auto it : this->path){
        ret += sqrt(it.x * it.x + it.y * it.y);
    }
    ret += abs(this->get_v()) * (ros::Time::now() - this->starttime).toSec();
    return ret;
}
void PoseTracer::reset()
{
    // this->dist = 0;
    this->path.clear();
}
void PoseTracer::clear()
{
    this->coor = {};
    // this->vel = {};
    // this->dist = 0;
    this->path.clear();
    this->starttime = ros::Time::now();
    ROS_INFO("PoseTracer cleared");
}
void PoseTracer::set_vw(int _v, int _w)
{
    // different op
    if(_v != this->get_v() || _w != this->get_w()){
        // don't record redundant operation
        if(this->get_v() || this->vel.w){
            double t = (ros::Time::now() - this->starttime).toSec();
            // this->dist += abs(this->get_v() * t); // may have bug
            this->coor.x += this->vel.x * t;
            this->coor.y += this->vel.y * t;
            this->coor.w += this->vel.w * t;
            this->coor.w = this->roundPi(this->coor.w);
            geometry_msgs::Quaternion tmp;
            tmp.x = this->vel.x * t;
            tmp.y = this->vel.y * t;
            tmp.z = this->vel.z * t;
            tmp.w = this->vel.w * t;
            tmp.w = this->roundPi(tmp.w);
            this->path.push_back(tmp);
            // clear
            this->vel = {};
        }
        this->vel.x = _v * cos(this->coor.w);
        this->vel.y = _v * sin(this->coor.w);
        this->vel.w = _w;
        this->v = _v;
        this->starttime = ros::Time::now();
    }
}