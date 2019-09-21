#ifndef UART_H
#define UART_H
#include <cmath>
#include <functional>
#include "pybot.h"

using namespace std;
using namespace std::placeholders;
using namespace pybot;
using namespace tircgo_uart;
namespace pybot
{
    class UART
    {
    public:
        UART(const string& _paremt_frame_id);
        ~UART();
        // may return invalid srv response
        RobotInvoke invoke(const char _op, const std::vector<int16_t>& _args);
        Tracking_status get_tracking_status()const;
        // void drive(const double _v, const double _w, const string& _frame)const;
    private:
        ros::NodeHandle n;
        void status_callback(const RobotStatus::ConstPtr& _msg);
        ros::ServiceClient invoke_clt; // send uart request
        ros::Publisher velCmd_pub; // drive AGV
        ros::Subscriber status_sub; // subscribe to status

        const string frame_id;
        Mode mode;
        Tracking_status tracking_status;
        int16_t lidar_levels[4];
        // The above variables will all be up to date
    };
}
#endif