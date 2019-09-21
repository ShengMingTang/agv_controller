#include "UART.h"
void UART::status_callback(const RobotStatus::ConstPtr& _msg)
{
    if(_msg->is_activated){
        ROS_INFO("Msg receiced");
        this->mode = (Mode)_msg->now_mode;
        if(_msg->jreply.is_activated){
            this->tracking_status = (Tracking_status)_msg->jreply.reply;
        }
        else{
            ROS_WARN("Jreply invalid");
        }
        if(_msg->lreply.is_activated){
            size_t dir = _msg->lreply.dir_reply - 1;
            int16_t level = _msg->lreply.level_reply;
            this->lidar_levels[dir] = level;
        }
        else{
            ROS_WARN("Lreply invalid\n");
        }
    }
    else{
        ROS_WARN("Msg invalid\n");
    }
}
UART::UART(const string& _parent_frame_id):
invoke_clt{this->n.serviceClient<RobotInvoke>(ROBOTINVOKE_TOPIC)},
status_sub{this->n.subscribe(ROBOTSTATUS_TOPIC, MSG_QUE_SIZE, &UART::status_callback, this)},
velCmd_pub{this->n.advertise<geometry_msgs::TwistStamped>(ROBOTVELCMD_TOPIC, MSG_QUE_SIZE)},
frame_id{_parent_frame_id + "/uart"}
{
    ROS_INFO("UART constructed\n");
}
RobotInvoke UART::invoke(const char _op, const std::vector<int16_t>& _args)
{
    RobotInvoke srv;
    srv.request.header.stamp = ros::Time::now();
    srv.request.header.frame_id = this->frame_id;
    srv.request.operation = _op;
    srv.request.argument = _args;
    if(!this->invoke_clt.call(srv)){
        ROS_ERROR("Failed to call invoke service\n");
    }
    return srv;
}
Tracking_status UART::get_tracking_status()const
{
    return this->tracking_status;
}
UART::~UART()
{
    ROS_INFO("UART destroyed\n");
}
// void UART::drive(const double _v, double _w, const string& _frame)
// {
//     if(abs(_v) <= MOTOR_LINEAR_LIMIT && abs(_w) <= MOTOR_ANGULAR_LIMIT){
//         RobotStatus msg;
//         msg.header.time_stamp = ros::Time::now();
//         msg.header.frame_id = _frame;
        
//         msg.twist.linear.x = _v;
//         msg.twist.linear.y = msg.twist.linear.z = 0;
        
//         msg.twist.angular.z = _w;
//         msg.twist.angular.x = msg.twist.angular.y = 0;
//         this->velCmd_pub.publish(msg);
//     }
//     else{
//         ROS_ERROR("Drive msg broadcasting rejected with v = %lf, w = %lf\n", _v, _w);
//     }
// }