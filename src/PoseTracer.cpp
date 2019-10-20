#include "PoseTracer.h"
#include <cmath>
PoseTracer::PoseTracer():
starttime{ros::Time::now()}
{
    this->coor.x = this->coor.y = this->coor.z = this->coor.w = 0;
    this->vel.x = this->vel.y = this->vel.z = this->vel.w = 0;
    ROS_INFO("PoseTracer constructed");
}
PoseTracer::~PoseTracer()
{
    ROS_INFO("PoseTracer destructed");
}
void PoseTracer::clear()
{
    this->reset_path();
    this->coor = {};
    ROS_INFO("PoseTracer cleared");
}
void PoseTracer::reset_path()
{
    int v = this->v, w = this->w;
    // reset
    this->set_vw(0, 0);
    this->path.clear();
    // keep vel
    this->set_vw(v, w);
}
void PoseTracer::set_vw(int _v, int _w)
{
    // if(_v != this->v || _w != this->w){ // different op
    if(this->v || this->w){ // won't record redundant operation
        double t = (ros::Time::now() - this->starttime).toSec();
        geometry_msgs::Quaternion motion;
        motion.x = this->vel.x * t;
        motion.y = this->vel.y * t;
        motion.w = this->vel.w * t;
        motion.w = roundPi(motion.w);
        this->path.push_back(motion);
        // update coor
        this->coor.x += motion.x;
        this->coor.y += motion.y;
        this->coor.w += motion.w;
        this->coor.w = roundPi(this->coor.w);
        this->vel = {};
    }
    // coor purpose
    this->vel.x = _v * cos(this->coor.w);
    this->vel.y = _v * sin(this->coor.w);
    this->vel.w = _w;
    // main part
    this->v = _v;
    this->w = _w;
    this->starttime = ros::Time::now();
    // }
}
geometry_msgs::Quaternion PoseTracer::get_coor()const
{
    geometry_msgs::Quaternion pos;
    double t = (ros::Time::now() - this->starttime).toSec();
    pos.x = this->coor.x + this->vel.x * t;
    pos.y = this->coor.y + this->vel.y * t;
    pos.w = this->coor.w + this->vel.w * t;
    pos.w = roundPi(pos.w);
    return pos;
}
double PoseTracer::get_dist()
{
    double dist = 0, t = (ros::Time::now() - this->starttime).toSec();
    for(auto it : this->path){
        dist += sqrt(it.x * it.x + it.y * it.y);
    }
    dist += abs(this->v * t);
    return dist;
}
double pybot::roundPi(double _w)
{
    while(_w < 0)
        _w += 2 * PI;
    while(_w >= 2 * PI)
        _w -= 2 * PI;
    return _w;
}