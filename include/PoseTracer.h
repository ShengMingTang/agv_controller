#ifndef POSE_TRACER_H
#define POSE_TRACER_H
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>
#include <list>
#include "pybot.h"
#define PI 3.14159

using namespace std;
using namespace pybot;
namespace pybot
{
    class PoseTracer
    {
    public:
        PoseTracer();
        ~PoseTracer();
        void clear();
        void reset_path(); // but keep vel
        void set_vw(int _v, int _w);
        // getter and setter
        int16_t get_v()const {return this->v;}
        int16_t get_w()const {return this->w;}
        geometry_msgs::Quaternion get_coor()const;
        double get_dist();
        list<geometry_msgs::Quaternion> get_path()const {return this->path;}
        ros::Time get_starttime()const {return this->starttime;}
    private:
        geometry_msgs::Quaternion coor = {}, vel = {};
        ros::Time starttime;
        
        // piecewise integration
        int16_t v, w;
        list<geometry_msgs::Quaternion> path; // consisting of op vectors
    };
    double roundPi(double _w);
}
#endif