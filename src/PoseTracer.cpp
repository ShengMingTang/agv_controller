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
    this->starttime = ros::Time::now();
}
void PoseTracer::stop()
{
    double t = (ros::Time::now() - this->starttime).toSec();
    this->is_driving = false;
    this->dist += abs(this->v * t);
    this->pose.orientation.x += this->velocity.orientation.x * t;
    this->pose.orientation.y += this->velocity.orientation.y * t;
    this->pose.orientation.w += this->velocity.orientation.w * t;
    // clear
    this->v = this->w = 0;
    this->velocity.orientation.x = this->velocity.orientation.y = this->velocity.orientation.w = 0;
}
double PoseTracer::get_dist()
{
    return this->dist + this->v * (ros::Time::now() - this->starttime).toSec();
}
void PoseTracer::reset()
{
    this->dist = 0;
}
void PoseTracer::set_vw(double _v, double _w)
{
    if(!this->is_driving){
        this->is_driving = true;
        // rotation first, one operation at a time
        if(_w != 0 )
            _v = 0;
        this->v = _v;
        this->w = _w;
        this->velocity.orientation.x = _v * cos(this->pose.orientation.w);
        this->velocity.orientation.y = _v * sin(this->pose.orientation.w);
        this->velocity.orientation.w = _w / 10;
    }
    else{
        ROS_WARN("double moving instruction, rejected, plz stop it first");
    }
}