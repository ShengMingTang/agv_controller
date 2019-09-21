#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <ros/ros.h>
#include <vector>
#include "pybot.h"
#include "Joystick.h"
#include "UART.h"
// #include "Wifi.h"
using namespace std;
using namespace pybot;
namespace pybot
{
    class Controller
    {
    friend class UART;
    public:
        Controller(const string& _id);
        void setup();
        void loop();
        ~Controller();
    private:
        ros::NodeHandle n;
        /* UART controlled */
        UART uart;
        const string frame_id;
        // Mode mode = Mode::MODE_IDLE;
        // Tracking_status tracking_status;
        // vector<int16_t> lidar_levels(4);

        // Joystickjoystick joystick;
        // Wifi wifi;
    };
}
#endif