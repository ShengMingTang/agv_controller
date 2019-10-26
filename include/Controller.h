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
// #include "Wifi.h"
#include "RouteNodeGraph.h"

#include "tircgo_uart/RobotStatus.h" // topic header for subscribing to the robot status
#include "tircgo_uart/RobotInvoke.h"
#include "tircgo_msgs/WifiNodeOcp.h"
#include "tircgo_msgs/WifiNodeCost.h"
#include "tircgo_msgs/WifiTaskConfirm.h"
#include "tircgo_msgs/Ask_Data.h"
#if AGV_CONTROLLER_TEST
    static int16_t route_ct, node_ct;
#endif


using namespace std;
using namespace pybot;
using namespace tircgo_uart;
using namespace tircgo_msgs;

using Node = tircgo_msgs::RouteNode; // for generality

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
        // Node get_nodeocp() const{return this->nd_ocp;}
        // float get_cost_to_target (const Node &_target)const;
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
        // Wifi wifi;
        bool is_target_ocp(const Node& _target);
        ros::ServiceServer nodeocp_srv;
        ros::ServiceClient nodeocp_clt;
        bool nodeocp_serve(WifiNodeOcp::Request &_req, WifiNodeOcp::Response &_res);

        ros::ServiceServer nodecost_srv;
        ros::ServiceClient nodecost_clt;
        bool nodecost_serve(WifiNodeCost::Request &_req, WifiNodeCost::Response &_res);

        ros::ServiceServer task_confirm_srv;
        bool task_confirm_serve(WifiTaskConfirm::Request &_req, WifiTaskConfirm::Response &_res);

        ros::ServiceServer askdata_srv;
        bool askdata_serve(Ask_Data::Request &_req, Ask_Data::Response &_res);
        /* graph */
        Graph<Node> graph;
        
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
        Node nd_training;
        // working related
        Node nd_target;
        Node nd_ocp;
        // Node with .route == .node == -1 is considered invalid
        list<Node> work_list;
        // below will be tracked if AGV_CONTROLLER_UART_DOMIN is On
        Tracking_status tracking_status = Tracking_status::TRACKING_STATUS_NONE;
        vector<int16_t> lidar_levels;

        /* not necessary */
        ros::Publisher monitor;
    };
}
#endif