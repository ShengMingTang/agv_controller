#include "PoseTracer.h"
#include <cmath>
PoseTracer::PoseTracer()
{
    ROS_INFO("PoseTracer constructed");
}
PoseTracer::~PoseTracer()
{
    ROS_INFO("PoseTracer destructed");
}
void PoseTracer::start()
{
    // this->starttime = ros::Time::now();
    // ROS_WARN("double moving instruction, rejected, plz stop it first");
}
void PoseTracer::stop()
{
    // double t = (ros::Time::now() - this->starttime).toSec();
    // this->dist += abs(this->get_v() * t); // may have bug
    // this->coor.x += this->vel.x * t;
    // this->coor.y += this->vel.y * t;
    // this->coor.w += this->vel.w * t;
    // geometry_msgs::Quaternion tmp;
    // tmp.x = this->vel.x * t;
    // tmp.y = this->vel.y * t;
    // tmp.z = this->vel.z * t;
    // tmp.w = this->vel.w * t;
    // this->path.push_back(tmp);
    // // clear
    // this->vel = {};
}
geometry_msgs::Quaternion PoseTracer::get_coor()const
{
    geometry_msgs::Quaternion tmp;
    tmp.x = this->coor.x + this->vel.x * (ros::Time::now() - this->starttime).toSec();
    tmp.y = this->coor.y + this->vel.y * (ros::Time::now() - this->starttime).toSec();
    // tmp.z = this->coor.z + this->vel.z * (ros::Time::now() - this->starttime).toSec();
    tmp.w = this->coor.w + this->vel.w * (ros::Time::now() - this->starttime).toSec();
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
    this->dist = 0;
    this->path.clear();
}
void PoseTracer::clear()
{
    this->coor = {};
    this->vel = {};
    this->dist = 0;
    this->path.clear();
    ROS_INFO("PoseTracer cleared");
}
void PoseTracer::set_vw(int _v, int _w)
{
    // different op
    if(_v != this->get_v() || _w != this->get_w()){
        // don't record redundant operation
        if(this->get_v() || this->vel.w){
            double t = (ros::Time::now() - this->starttime).toSec();
            this->dist += abs(this->get_v() * t); // may have bug
            this->coor.x += this->vel.x * t;
            this->coor.y += this->vel.y * t;
            this->coor.w += this->vel.w * t;
            geometry_msgs::Quaternion tmp;
            tmp.x = this->vel.x * t;
            tmp.y = this->vel.y * t;
            tmp.z = this->vel.z * t;
            tmp.w = this->vel.w * t;
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