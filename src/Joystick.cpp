#include "Joystick.h"
void Joystick::callback(const sensor_msgs::Joy::ConstPtr& _msg)
{
    this->ptr = _msg;
}
Joystick::Joystick(const string& _parent_frame_id):
frame_id{_parent_frame_id + "/joystick"},
sub{this->n.subscribe(JOYSTICKIO_TOPIC, MSG_QUE_SIZE, &Joystick::callback, this)}
{
    ROS_INFO("Joystick built\n");
}
sensor_msgs::Joy::ConstPtr Joystick::pop()
{
    return this->ptr;
}
Joystick::~Joystick()
{
    ROS_INFO("Joystick destroyed\n");
}