#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>

#include <vector>
#include <sstream>
#include <string>
#include <queue>
#include <cmath>
#include <list>
#include <map>

#include "Control_proto.h"
#include "Joystick.h"
#include "UART.h"
#include "PoseTracer.h"
#include "Graph.h"
#include "tircgo_common.h"

#include "tircgo_uart/RobotStatus.h" // topic header for subscribing to the robot status
#include "tircgo_uart/RobotInvoke.h"

#include "tircgo_msgs/WifiNodeOcp.h"
#include "tircgo_msgs/WifiNodeCost.h"
#include "tircgo_msgs/WifiTaskConfirm.h"
#include "tircgo_msgs/Ask_Data.h"
#include "tircgo_msgs/CtrlData.h"
#include "tircgo_msgs/RouteNode.h"

#define RUNTIME_VARS_SET 1
#define RUNTIME_VARS_RESET 0

#define MODE_IDLE 1
#define MODE_POS 2
#define MODE_CALIB 4
#define MODE_TRAINING 8
#define MODE_WORKING 16
#define MODE_AUTO 1 << 14
#define MODE_NOTOK 1 << 15

#define TRAIN_ROUTE_MAX 5
#define TRAIN_NODE_MIN 2
#define TRAIN_NODE_MAX 10000

#define DRIVE_VEL_LINEAR 30
#define DRIVE_VEL_ANGULAR 5

#define CONTROL_WIFI 1

using namespace std;
using namespace tircgo;
using namespace tircgo_uart;
using namespace tircgo_msgs;

using PrimitiveType = tircgo_msgs::RouteNode;
using WalkUnitType = WalkUnit;
using VertexType = Vertex<PrimitiveType>;
using EdgeType = Edge<VertexType, WalkUnitType>;

namespace tircgo
{
    class Controller
    {
    public:
        Controller(const string& _id);
        ~Controller();
        void setup(int _argc, char **argv);
        void loopOnce(); // maybe function queue drivable
        bool ok() const{return !(this->stage_bm & MODE_NOTOK);}
        void monitor_display()const;
        string dumps_graph();
    private:
        /* Mode related */
        void sleep();
        void idle();
        void calibration();
        void training();
        void working();
        /* API */
        // custom working list
        /* return true if there is any kerenl instrcution*/
        bool is_target_ocp(const VertexType *vptr);
        bool priviledged_instr();
        bool shutdown();
        bool set_node();
        bool drive(vector<int16_t> _vel);
        void runtime_vars_mgr(bool _flag);

        /* API suport */
        int16_t decode_opcode();
        vector<int16_t> decode_drive();
        
        /* Sys related */
        void clear(); // print only
        void log(); // print only
        bool check_safety(); // tell near an obstable only
        sensor_msgs::Joy::ConstPtr get_joy_signal(); // Joy pub
        
        /* build time */
        ros::NodeHandle n;
        const string frame_id;
        int control = 0;
        ros::Rate loop_rate = ros::Rate(20);
        float drive_timeout = 0.4;
        int close_enough = 60;
        
        /* UART related */
        UART base_driver;
        void status_tracking(const RobotStatus::ConstPtr& _msg); // UART pub
        ros::Subscriber tracking_status_sub; // subscribe to status
        PoseTracer pose_tracer; // driving accumulator

        /* Joystick related */
        Joystick joystick;

        /* wifi related */
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
        
        /* Graph related */
        Graph<VertexType, EdgeType> graph;
        vector< vector<VertexType*> > rn_img; // [route][node] -> reference in graph

        /* runtime supoort vars */
        sensor_msgs::Joy::ConstPtr op_ptr;
        int mode = MODE_IDLE; // strictly tracked
        int stage_bm = MODE_IDLE;
        vector<int16_t> lidar_levels;
        ros::Publisher monitor;
        vector<int16_t> op_vel;
        int16_t op = OPCODE_NONE;
        
        // training and working
        int16_t training_route = 0, training_node = 0;
        PrimitiveType nd_training;
        // working related
        PrimitiveType nd_target; // for manually set target
        VertexType *target_vptr;
        VertexType *ocp_vptr;
        
        list<VertexType*> work_list;
        
        // strictly tracked
        int16_t tracking_status = TRACKING_STATUS_NONE;
    };
}
#endif