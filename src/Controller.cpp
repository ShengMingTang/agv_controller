#include "Controller.h"
Controller::Controller(const string& _id):
uart{UART(_id)}
{
    ROS_INFO("Controller constructed\n");
}
void Controller::setup()
{

}
void Controller::loop()
{

}
Controller::~Controller()
{
    ROS_INFO("Controller destructed\n");
}