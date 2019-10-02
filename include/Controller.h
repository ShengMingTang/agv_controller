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
// #include "wifi.h"
#include "Proc.h"
#include "ProcTodo.h"
#include "UART.h"
#include "PoseTracer.h"
#include "tircgo_uart/RobotStatus.h" // topic header for subscribing to the robot status
#include "RouteNodeGraph.h"
#include "wifi/RouteNode.h"
#include "RouteNodeGraph.h"

#define CONTROLLER_VERBOSE 1
#define CONTROLLER_TEST 1
#if CONTROLLER_TEST
    static int16_t route_ct, node_ct;
#endif
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
        bool ok() const{return this->is_ok;}
    private:
        /* new version */
        template<typename T>
        void exec(Ptr<T> _ptr);
        /* end */
        bool check_safety(); //
        void idle();
        void homing();
        void training();
        void working(); //
        void clear(); //
        void log(); //
        void drive();
        void set_node(int16_t _route, int16_t _node, double _w); //
        /* op related */
        Opcode decode_opcode(sensor_msgs::Joy::ConstPtr& _ptr);
        void decode_drive(sensor_msgs::Joy::ConstPtr& _ptr); //
        sensor_msgs::Joy::ConstPtr get_joy_signal();
        
        /* ISR */
        void isr();
        /* build time */
        ros::NodeHandle n;
        const string frame_id;
        
        /* UART related */
        UART base_driver;
        ros::Subscriber tracking_status_sub; // subscribe to status
        void status_tracking(const RobotStatus::ConstPtr& _msg);
        // these will be tracked
        Tracking_status tracking_status = Tracking_status::TRACKING_STATUS_NONE;
        vector<int16_t> lidar_levels = vector<int16_t>(4);

        /* Joystick */
        Joystick joystick;
        
        // Wifi wifi;

        /* runtime */
        // AGV-wise parameter
        Mode mode = Mode::MODE_IDLE;
        bool is_origin_set = false;
        bool is_calibed = false;
        bool is_trained = false;
        bool is_calib_begin = false;
        bool is_ok = true;
        /* driving accumulator */
        // geometry_msgs::Pose pose; // position, orientation
        PoseTracer pose_tracer;


        Graph<wifi::RouteNode> graph;
        ProcTodo tasks;
        Opcode op = Opcode::OPCODE_NONE;

        // training related
        int training_route, training_node;
        // working related
        int target_route, target_node;
        int working_route, working_node;


        /* Test */
        #if CONTROLLER_VERBOSE
            ros::Publisher monitor;
            void monitor_display();
        #endif
    };
}
#endif