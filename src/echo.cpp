#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("%s", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "echo");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("debug_topic", 100, chatterCallback);
  ros::spin();

  return 0;
}
