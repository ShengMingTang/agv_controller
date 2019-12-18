#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/String.h>

#include <vector>
#include <sstream>
#include <fstream>
#include <string>
#include <queue>
#include <cmath>
#include <list>
#include <map>
#include <climits>

#include "Control_proto.h"
#include "Joystick.h"
#include "UART.h"
#include "PoseTracer.h"
#include "Graph.h"
#include "tircgo_common.h"

#include "tircgo_uart/RobotStatus.h"
#include "tircgo_uart/RobotInvoke.h"

#include "tircgo_msgs/WifiNodeOcp.h"
#include "tircgo_msgs/WifiNodeCost.h"
#include "tircgo_msgs/WifiTaskConfirm.h"
#include "tircgo_msgs/Ask_Data.h"
#include "tircgo_msgs/CtrlData.h"
#include "tircgo_msgs/RouteNode.h"

#include <actionlib/server/simple_action_server.h>
#include "tircgo_controller/scheduleAction.h"
#include "tircgo_msgs/ControllerTalk.h"

#define MODE_IDLE (int16_t)1
#define MODE_POS (int16_t)2
#define MODE_CALIB (int16_t)4
#define MODE_TRAINING (int16_t)8
#define MODE_WORKING (int16_t)16
#define MODE_AUTO (int16_t)32
#define MODE_AGENT (int16_t)64
#define MODE_NOTOK (int16_t)128

#define TRAIN_ROUTE_MAX 5
#define TRAIN_NODE_MIN 2
#define TRAIN_NODE_MAX 10000

#define DRIVE_VEL_LINEAR 40
#define DRIVE_VEL_ANGULAR 8

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
        Controller(const string& _id = "");
        ~Controller();
        void setup(int _argc, char **argv);
        void loopOnce(); // maybe function queue drivable
        bool ok() const{return !(this->stage_bm & MODE_NOTOK);}
        void monitor_display()const;
        string dumps_graph();
    private:
        /* ---------------------------------------------------------------------------------------------*/

        /* API */
        bool drive(vector<int16_t> _vel);
        bool priviledged_instr(int16_t _op);
        bool shutdown();
        
        
        bool train_begin();
        bool set_node();
        bool train_finish();

        bool add_target(const RouteNode &_nd);
        bool trigger_working();
        
        /* API suport */
        bool is_target_ocp(const VertexType *vptr);
        bool is_target_valid(const RouteNode &_nd);

        /* ---------------------------------------------------------------------------------------------*/
        
        /* Internal Call */

        /* operation variation */
        int16_t decode_opcode(sensor_msgs::Joy::ConstPtr _ptr);
        vector<int16_t> decode_drive(sensor_msgs::Joy::ConstPtr _ptr);
        /* Mode related */
        void idle(const int16_t &_op);
        void calibration(const int16_t &_op);
        void training(const int16_t &_op);
        bool working(const int16_t &_op);
        /* Sys related */
        void clear();
        void log();
        void restore(fstream &fs);
        bool check_safety(vector<int16_t> _vel); // tell if near an obstable only

        /* ---------------------------------------------------------------------------------------------*/
        
        /* build time */
        ros::NodeHandle n;
        string frame_id;
        string timestamp;
        string log_path; // log file path, search and log
        int control = 0;
        ros::Rate loop_rate = ros::Rate(5);
        float drive_timeout = 0.4;
        int close_enough = 30;

        /* runtime supoort vars */
        int16_t mode = MODE_IDLE; // strictly tracked
        int16_t tracking_status = TRACKING_STATUS_NONE; // strictly tracked
        vector<int16_t> lidar_levels; // strictly tracked
        /* auto */
        void execute_schedule(const tircgo_controller::scheduleGoalConstPtr &_goal);
        actionlib::SimpleActionServer<tircgo_controller::scheduleAction> sch_srv;
        // ControllerTalk
        ros::Publisher pub_agent_imm;
        tircgo_controller::scheduleFeedback sch_feedback;
        tircgo_controller::scheduleResult sch_res;
        boost::mutex train_mutex;
        boost::mutex work_mutex;

        int16_t stage_bm = MODE_IDLE;
        ros::Publisher monitor;
        
        /* Graph related */
        Graph<VertexType, EdgeType> graph;
        string graph_viz;
        vector< vector<VertexType*> > rn_img; // [route][node] -> reference in graph
        const RouteNode get_affinity_vertex(const VertexType* _curr, const VertexType* _vptr);
        /* training and working */
        int16_t training_route = 0, training_node = 0;
        PrimitiveType nd_training;
        // working related
        PrimitiveType nd_target; // for manually set target
        VertexType *ocp_vptr;
        list<VertexType*> work_list;
        
        /* ---------------------------------------------------------------------------------------------*/

        /* UART related */
        UART base_driver;
        void status_tracking(const RobotStatus::ConstPtr& _msg); // UART pub
        ros::Subscriber tracking_status_sub; // subscribe to status
        PoseTracer pose_tracer; // driving accumulator

        /* Joystick related */
        Joystick joystick;
        sensor_msgs::Joy::ConstPtr get_joy_signal(); // Joy subb

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
    };
}
#endif