#ifndef UART_H
#define UART_H
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <functional>
#include <vector>
#include "pybot.h"
#include "tircgo_uart/RobotInvoke.h" // srv header

#define UART_VERBOSE 0
using namespace std;
using namespace pybot;
using namespace tircgo_uart;
namespace pybot
{
    class UART
    {
    public:
        UART(const string& _paremt_frame_id);
        ~UART();
        RobotInvoke invoke(const char _op, const std::vector<int16_t>& _args);
        bool is_invoke_valid(RobotInvoke _srv);
    private:
        ros::NodeHandle n;
        const string frame_id;

        ros::ServiceClient invoke_clt; // send uart request
        // ros::Publisher velCmd_pub; // drive AGV
    };
}
#endif