#ifndef UART_H
#define UART_H
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <functional>
#include <vector>
#include <sstream>
#include <list>
#include "Control_proto.h"
#include "tircgo_uart/RobotInvoke.h" // srv header
#include "tircgo_controller/scheduleAction.h" // action header

using namespace std;
using namespace tircgo;
using namespace tircgo_uart;
using namespace tircgo_controller;

namespace tircgo
{
    class UART
    {
    public:
        UART(const string& _parent_frame_id);
        ~UART();
        RobotInvoke invoke(const int16_t _op, std::vector<int16_t> _args);
        bool is_invoke_valid(const RobotInvoke &_srv);
        string get_cmds();
        void clear();
    private:
        ros::NodeHandle n;
        const string frame_id;

        ros::ServiceClient invoke_clt; // send uart request
        list<tircgo_controller::scheduleActionGoal::_goal_type> cmds;

        ros::Time lastdriveTime;
    };
}
#endif