#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <vector>
#include <sstream>
#include <string>
#include <queue>
#include <cmath>
#include <list>
#include "Control_proto.h"
#include "Joystick.h"
#include "UART.h"
#include "PoseTracer.h"
#include "Wifi.h"
#include "RouteNodeGraph.h"

#include "tircgo_uart/RobotStatus.h" // topic header for subscribing to the robot status
#include "tircgo_uart/RobotInvoke.h"
#include "tircgo_msgs/WifiNodeOcp.h"

#if AGV_CONTROLLER_TEST
    static int16_t route_ct, node_ct;
#endif


using namespace std;
using namespace pybot;
using namespace tircgo_uart;
using namespace tircgo_msgs;
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
        void isr(const Tracking_status& _cond); // interrupt service routine
        void idle();
        void homing();
        void training();
        void working(); // print only

        /* Drive related */
        Opcode decode_opcode(sensor_msgs::Joy::ConstPtr& _ptr);
        pair<int16_t, int16_t> decode_drive(sensor_msgs::Joy::ConstPtr& _ptr); //
        void drive();
        
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

        /* Joystick related */
        Joystick joystick;
        
        /* wifi related */
        Wifi wifi;
        ros::ServiceServer nodeocp_srv;
        bool wifi_nodeocp_serve(WifiNodeOcp::Request &req, WifiNodeOcp::Response &res);
        // ask for node_ocp
        // service to take a job
        // provide any info needed by Wifi communication

        /* graph */
        Graph<tircgo_msgs::RouteNode> graph;
        
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
        int16_t training_route = 0, training_node = 0;
        // working related
        RouteNode target_rn;
        RouteNode node_ocp;
        RouteNode rn_none;
        list<RouteNode> work_list;
        // below will be tracked if AGV_CONTROLLER_UART_DOMIN is On
        Tracking_status tracking_status = Tracking_status::TRACKING_STATUS_NONE;
        vector<int16_t> lidar_levels;

        /* not necessary */
        ros::Publisher monitor;
    };
}
#endif