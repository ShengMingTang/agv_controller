#include "PoseTracer.h"
#include <cmath>
PoseTracer::PoseTracer():
starttime{ros::Time::now()}
{
    this->coor.x = this->coor.y = this->coor.z = this->coor.w = 0;
    this->vel = {0, 0};
    // ROS_INFO("[PoseTracer] constructed");
}
PoseTracer::~PoseTracer()
{
    // ROS_INFO("[PoseTracer] destructed");
}
void PoseTracer::clear()
{
    this->reset_path();
    this->coor = {};
    ROS_INFO("[PoseTracer] cleared");
}
void PoseTracer::reset_path()
{
    vector<int16_t> tmp = this->vel;
    vector<int16_t> zero = {0, 0};
    // reset
    this->set_vw(zero);
    this->path.clear();
    // keep vel
    this->set_vw(tmp);
}
void PoseTracer::set_vw(const vector<int16_t> &_vel) //const vector<int16_t> &_vel
{
    if(_vel[0] != this->vel[0] || _vel[1] != this->vel[1]){ // different op
        // record a piece of walkunit
        ros::Time now = ros::Time::now();
        double t = (now - this->starttime).toSec();
        WalkUnitType unit;

        if(this->vel[0] || this->vel[1]){ // non-redundant move
            unit.vel = this->vel;
            unit.dur = now - this->starttime;
            this->path.push_back(unit);
            // update coor
            this->coor.x += this->vel[0] * cos(this->coor.w) * t;
            this->coor.y += this->vel[0] * sin(this->coor.w) * t;
            this->coor.w += (this->vel[1] * ANGULAR_FACTOR) * t;
            this->coor.w = roundPi(this->coor.w);
        }
        this->vel = _vel;
        this->starttime = ros::Time::now();
    }
}
geometry_msgs::Quaternion PoseTracer::get_coor()const
{
    geometry_msgs::Quaternion pos;
    double t = (ros::Time::now() - this->starttime).toSec();
    pos.x = this->coor.x + this->vel[0] * cos(this->coor.w) * t;
    pos.y = this->coor.y + this->vel[0] * sin(this->coor.w) * t;
    pos.w = roundPi(this->coor.w + (this->vel[1] * 0.1) * t);
    return pos;
}
double PoseTracer::get_dist()
{
    double dist = 0;
    double t = (ros::Time::now() - this->starttime).toSec();
    for(auto it : this->path){
        dist += abs(it.vel[0]) * it.dur.toSec();
    }
    dist += abs(this->vel[0] * t);
    return dist;
}
/*  return the headway of the last linear motion */
int16_t PoseTracer::get_headway()const
{
    for(auto it = this->path.crbegin(); it != this->path.crend(); it++){
        if((it->vel)[0]> 0){
            return SETNODE_HEADWAY_HEAD;
        }
        else if((it->vel)[0] < 0){
            return SETNODE_HEADWAY_TAIL;
        }
    }
    return SETNODE_HEADWAY_HEAD;
}
double tircgo::roundPi(double _w)
{
    while(_w < 0)
        _w += 2 * PI;
    while(_w >= 2 * PI)
        _w -= 2 * PI;
    return _w;
}
