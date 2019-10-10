#include "Joystick.h"
Joystick::Joystick(const string& _parent_frame_id)
:frame_id {_parent_frame_id + "/joystick"}
,sub {this->n.subscribe(JOYSTICKIO_TOPIC, MSG_QUE_SIZE, &Joystick::callback, this)}
,last_press_t {ros::Time::now()}
{
    ROS_INFO("Joystick constructed");
}
Joystick::~Joystick()
{
    ROS_INFO("Joystick destroyed");
}
sensor_msgs::Joy::ConstPtr Joystick::pop()
{
    while(!this->que.empty()){
        auto sig = this->que.front();
        float t = (sig->header.stamp - this->last_press_t).toSec();
        if(t >= JOY_CD){ //valid
            this->last_press_t = sig->header.stamp;
            this->que.pop();
            return sig;
        }
        else{ // else take it as bouncing, ignored
            this->que.pop();
        }
    }
    return nullptr;
}
void Joystick::callback(const sensor_msgs::Joy::ConstPtr& _msg)
{
    while(this->que.size() >= JOY_BUFF_SIZE)
        this->que.pop();
    this->que.push(_msg);
}