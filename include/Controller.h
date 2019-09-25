#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/String.h>
#include <vector>
#include <set>
#include <map>
#include "pybot.h"
#include "Joystick.h"
#include "UART.h"
#include "tircgo_uart/RobotStatus.h" // topic header for subscribing to the robot status
#include <sstream>
#define CONTORLLER_VERBOSE 1
using namespace std;
using namespace pybot;
namespace pybot
{
    class Controller
    {
    public:
        Controller(const string& _id);
        ~Controller();
        void setup(); //
        void loopOnce();

    private:
        void idle();
        void homing();
        void training();
        void working(); //
        void clear(); //
        void log(); //
        void drive();
        void set_node(int16_t _node); //

        Opcode decode_opcode(sensor_msgs::Joy::ConstPtr& _ptr);
        void decode_drive(sensor_msgs::Joy::ConstPtr& _ptr); //
        sensor_msgs::Joy::ConstPtr get_joy_signal();

        ros::NodeHandle n;
        const string frame_id;
        Mode mode = Mode::MODE_IDLE;
        Tracking_status tracking_status;
        vector<int16_t> lidar_levels = vector<int16_t>(4);
        
        /* slaves */
        UART uart;
        ros::Subscriber tracking_status_sub; // subscribe to status
        void status_tracking(const RobotStatus::ConstPtr& _msg);
        map<int16_t, Mode> mode_tf = {
            {1, Mode::MODE_IDLE},
            {2, Mode::MODE_HOMING},
            {3, Mode::MODE_TRAINING},
            {4, Mode::MODE_WORKING}
        };

        Joystick joystick;
        // Wifi wifi;

        /* runtime */
        // sensor_msgs::Joy::ConstPtr op_ptr = nullptr;
        Opcode op;
        vector<int16_t> vw; // linear and angular velocity
        // Graph graph
        // AGV-wise parameter
        bool is_origin_set = false;
        bool is_calibed = false;
        bool is_trained = false;
        bool is_calib_begin = false;
        // implement graph

        // training related
        int training_route, training_node;
        // working related
        int working_route, working_node;

        /* debug */
        ros::Publisher debugger;
    };
}
#endif