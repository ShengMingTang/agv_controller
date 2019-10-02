#include "Joystick.h"
void Joystick::callback(const sensor_msgs::Joy::ConstPtr& _msg)
{
    // if(!this->is_frozen){
    //     this->ptr = _msg;
    //     this->is_frozen = true;
    //     this->timer.start();
    // }
    // else{ // frozen
    //     this->ptr = nullptr;
    // }
    float t = (_msg->header.stamp - this->last_press_t).toSec();
    if(t >= JOY_CD){ //valid
        this->ptr = _msg;
        this->last_press_t = _msg->header.stamp;
    }
    else{ // else take it as bouncing, ignored
        ROS_INFO("Joystick signal blocked");
        this->ptr = nullptr;
    }
}
Joystick::Joystick(const string& _parent_frame_id):
frame_id{_parent_frame_id + "/joystick"}
,sub{this->n.subscribe(JOYSTICKIO_TOPIC, MSG_QUE_SIZE, &Joystick::callback, this)}
,last_press_t{ros::Time::now()}
// ,timer{this->n.createTimer(ros::Duration(JOY_CD), &Joystick::timer_callback, this, true)}
{
    ROS_INFO("Joystick built\n");
}
sensor_msgs::Joy::ConstPtr Joystick::pop()
{
    auto ret = this->ptr;
    this->ptr = nullptr;
    return ret;
}
Joystick::~Joystick()
{
    ROS_INFO("Joystick destroyed\n");
}
void Joystick::timer_callback(const ros::TimerEvent& _event)
{
    // this->is_frozen = false;
    // ROS_INFO("Joy cd finished");
    // this->timer.stop();
    // this->timer.setPeriod(ros::Duration(JOY_CD));
}