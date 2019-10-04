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
    // struct geometry_msgs::Quaternion
    // {
    //     geometry_msgs::Quaternion(double _v, double _w, double _t)
    //     :v{_v}, w{_w}, t{_t} {}
    //     double v = 0;
    //     double w = 0;
    //     double t = 0;
    // };
    class PoseTracer
    {
    public:
        PoseTracer();
        ~PoseTracer();
        void start();
        void stop();
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
        ros::Time starttime;
        // piecewise integration
        int v, w;
        double dist = 0;
        list<geometry_msgs::Quaternion> path;
    };
}
#endif