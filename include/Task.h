#ifndef TASK_H
#define TASK_H
#include <ros/ros.h>
#include <queue>
#include "pybot.h"
namespace pybot
{
    class Task
    {
    public:
        Task();
        virtual queue<Task> expand() = 0; // expand task into smallers tasks
        virtual bool is_finished() = 0;
    private:
    };
}
#endif