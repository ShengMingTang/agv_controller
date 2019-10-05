#ifndef POSE_TRACER_H
#define POSE_TRACER_H
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>
#include <list>
#include "pybot.h"
using namespace std;
using namespace pybot;
namespace pybot
{
    class PoseTracer
    {
    public:
        PoseTracer();
        ~PoseTracer();
        void reset();
        void clear();
        // almost real-time distance
        void set_vw(int _v, int _w);
        int get_v()const {return this->v;}
        int get_w()const {return this->vel.w;}
        geometry_msgs::Quaternion get_coor()const;
        double get_dist();
        list<geometry_msgs::Quaternion> get_path()const {return this->path;}
    private:
        // ros::NodeHandle n;
        geometry_msgs::Quaternion coor, vel;
        double roundPi(double _w)const;
        ros::Time starttime;
        // piecewise integration
        int v, w;
        // double dist = 0;
        list<geometry_msgs::Quaternion> path;
    };
}
#endif