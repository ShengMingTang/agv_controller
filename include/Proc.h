#ifndef PROC_H
#define PROC_H
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <queue>
#include "pybot.h"
#include "Controller.h"
#include "wifi/RouteNode.h"
/*
    All jobs are defined in unit of Proc(Process),
    different jobs mean just different order of Proc.

    If want to add a job, just overload pybot::Controller::exec(T), T is a Proc,
    including all its derived Proc.
*/
using namespace pybot;
namespace pybot
{
    enum{
        PRIOR_INTER,
        PRIOR_WORK,
    };
    enum{
        PUSH_FRONT, // immediate
        PUSH_BACK
    };
    // push->get_front->exec->check_finished->pop
    struct Proc // base class
    {
        Proc(int _p, int _d):
        prior{_p},
        push_dir{_d}
        {}
        ~Proc() {}
        virtual queue< Ptr<Proc> > expand() = 0; // expand task into smallers tasks
        virtual void check_finished() = 0;
        const int prior = 0;
        const int push_dir = 0;
        bool is_finished = false;
    };
    struct JoyInter: Proc
    {
        JoyInter(sensor_msgs::Joy::ConstPtr _ptr, int _p, int _d):
        Proc(_p, _d),
        ptr{_ptr}
        {}
        virtual void check_finished();
        sensor_msgs::Joy::ConstPtr ptr;
    };
    struct Homing: Proc
    {
        Homing():
        Proc(PRIOR_WORK, PUSH_BACK)
        {}
        virtual void check_finished();
    };
    struct Training: Proc
    {
        Training():
        Proc(PRIOR_WORK, PUSH_BACK)
        {}
        virtual void check_finished();
    };
    struct Drive: Proc
    {
        Drive(double _v, double _w):
        Proc(PRIOR_WORK, PUSH_BACK),
        v{v}, w{_w}
        {}
        virtual void check_finished();
        double v, w;
    };
}
#endif