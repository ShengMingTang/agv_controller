#ifndef POSE_TRACER_H
#define POSE_TRACER_H
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <list>
#include "Control_proto.h"
#include "tircgo_common.h"
#define PI 3.14159

using namespace std;
using namespace tircgo;

using WalkUnitType = WalkUnit;

namespace tircgo
{
    class PoseTracer
    {
    public:
        PoseTracer();
        ~PoseTracer();
        void clear();
        void reset_path(); // but keep vel
        void set_vw(const vector<int16_t> &_vel);
        // getter and setter
        vector<int16_t> get_vel() const{return this->vel;}
        geometry_msgs::Quaternion get_coor()const;
        double get_dist();
        list<WalkUnitType> get_path()const {return this->path;}
        ros::Time get_starttime()const {return this->starttime;}
    private:
        geometry_msgs::Quaternion coor = {};
        ros::Time starttime;
        
        // piecewise integration
        // int16_t v = 0, w = 0;
        vector<int16_t> vel;
        list<WalkUnitType> path; // consisting of op vectors
    };
    double roundPi(double _w);
}
#endif