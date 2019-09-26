#include "Joystick.h"
void Joystick::callback(const sensor_msgs::Joy::ConstPtr& _msg)
{
    this->ptr = _msg;
    // int pressed = -1; 
    // for(int i = 0; i < _msg->buttons.size(); i++){
    //     if(_msg->buttons[i]){
    //         pressed = i;
    //         break;
    //     }
    // }
    // ROS_INFO("Frozen = %d", (int)this->is_frozen);
    // if(pressed == -1){
    //     return;
    // }
    // else{ //lock
    //     if(this->is_frozen && this->last_pressed == pressed){
    //         this->is_frozen = true;
    //         this->timer.start();
    //         this->ptr = nullptr;
    //     }
    //     this->last_pressed = pressed;
    // }
}
Joystick::Joystick(const string& _parent_frame_id):
frame_id{_parent_frame_id + "/joystick"}
,sub{this->n.subscribe(JOYSTICKIO_TOPIC, MSG_QUE_SIZE, &Joystick::callback, this)}
,timer{this->n.createTimer(ros::Duration(1), &Joystick::timer_callback, this, true)}
{
    ROS_INFO("Joystick built\n");
}
sensor_msgs::Joy::ConstPtr Joystick::pop()
{
    // auto ret = this->ptr;
    // this->ptr = nullptr;
    // return ret;
    return this->ptr;
}
Joystick::~Joystick()
{
    ROS_INFO("Joystick destroyed\n");
}
void Joystick::timer_callback(const ros::TimerEvent& _event)
{
    this->is_frozen = false;
    this->last_pressed = -1;
}