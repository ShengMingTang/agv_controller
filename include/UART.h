#ifndef UART_H
#define UART_H
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <functional>
#include <vector>
#include <sstream>
#include "Control_proto.h"
#include "tircgo_uart/RobotInvoke.h" // srv header

#define UART_VERBOSE 1
using namespace std;
using namespace tircgo;
using namespace tircgo_uart;
namespace tircgo
{
    class UART
    {
    public:
        UART(const string& _parent_frame_id);
        ~UART();
        RobotInvoke invoke(const int16_t _op, const std::vector<int16_t> _args);
        bool is_invoke_valid(RobotInvoke _srv);
    private:
        ros::NodeHandle n;
        const string frame_id;

        ros::ServiceClient invoke_clt; // send uart request
        // ros::Publisher velCmd_pub; // drive AGV
    };
}
#endif