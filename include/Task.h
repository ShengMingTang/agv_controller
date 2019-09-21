#include <ros/ros.h>
#include "pybot.h"
using namespace tircgo_uart;
class Task
{
public:
    Task();
    // expand task into smallers tasks
    virtual void expand() = 0;
    virtual bool is_finished() = 0;
private:
};