#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include <queue>
#include <cmath>
#include "pybot.h"
#include "Joystick.h"
#include "UART.h"
#include "PoseTracer.h"

#include "tircgo_uart/RobotStatus.h" // topic header for subscribing to the robot status
#include "tircgo_uart/RobotInvoke.h"

// #include "wifi/RouteNode.h"
// #include "Wifi.h"
// #include "RouteNodeGraph.h"

#if AGV_CONTROLLER_TEST
    static int16_t route_ct, node_ct;
#endif
using namespace std;
using namespace pybot;
using namespace tircgo_uart;
namespace pybot
{
    class Controller
    {
    public:
        Controller(const string& _id);
        ~Controller();
        void setup(); // stop @ here
        void loopOnce(); // maybe function queue drivable
        bool ok() const{return this->is_ok;}
    private:
        /* Mode related */
        void idle();
        void homing();
        void training();
        void working(); // print only

        /* Drive related */
        Opcode decode_opcode(sensor_msgs::Joy::ConstPtr& _ptr);
        pair<int16_t, int16_t> decode_drive(sensor_msgs::Joy::ConstPtr& _ptr); //
        void drive(); // stop @ here
        
        /* Sys related */
        void clear(); // print only
        void log(); // print only
        bool check_safety(); // tell near an obstable only
        void status_tracking(const RobotStatus::ConstPtr& _msg); // UART pub
        sensor_msgs::Joy::ConstPtr get_joy_signal(); // Joy pub
        void monitor_display();
        
        /* build time */
        ros::NodeHandle n;
        const string frame_id;
        
        /* UART related */
        UART base_driver;
        ros::Subscriber tracking_status_sub; // subscribe to status
        PoseTracer pose_tracer; // driving accumulator
        // bool drive_timeout = false; // false mean need to invoke again
        /* Joystick */
        Joystick joystick;
        
        // stop @ here
        // Wifi wifi;
        // Graph<wifi::RouteNode> graph;

        /* not necessary */
        ros::Publisher monitor;
        
        /* AGV-wise runtime parameter */
        Mode mode = Mode::MODE_IDLE;
        bool is_origin_set = false;
        bool is_calibed = false;
        bool is_trained = false;
        bool is_calib_begin = false;
        bool is_ok = true;
        sensor_msgs::Joy::ConstPtr op_ptr;
        Opcode op = Opcode::OPCODE_NONE;
        // training related
        int training_route, training_node;
        // working related
        int target_route, target_node;
        int working_route, working_node;
        // below will be tracked if AGV_CONTROLLER_UART_DOMIN is On
        Tracking_status tracking_status = Tracking_status::TRACKING_STATUS_NONE;
        vector<int16_t> lidar_levels;

    };
}
#endif