#ifndef POSE_TRACER_H
#define POSE_TRACER_H
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
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
        void start();
        void stop();
        void reset();
        // almost real-time distance
        void set_vw(double _v, double _w);
        double get_v()const {return this->v;}
        double get_w()const {return this->w;}
        double get_dist();
        geometry_msgs::Pose get_pose()const {return this->pose;}
    private:
        // ros::NodeHandle n;
        geometry_msgs::Pose pose, velocity; // velocity here represents 6 axes vector
        bool is_driving = false;
        bool is_cmd_sent = false;
        double v = 0, w = 0;
        ros::Time starttime;
        // piecewise integration
        double dist = 0;
    };
}
#endif