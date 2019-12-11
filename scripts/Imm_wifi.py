#! /usr/bin/env python
from __future__ import print_function
from enum import Enum
import rospy
import actionlib
import sys
import tircgo_controller.msg
import tircgo_msgs.msg

rospy.init_node('imm_wifi', anonymous=True)
pub = rospy.Publisher('robot_wifi_controller_talk_outer', tircgo_msgs.msg.ControllerTalk, queue_size=10)
def callback(data):
    pub.publish(data)
    rospy.loginfo('hi')
def Subscriber():
    sub = rospy.Subscriber('robot_wifi_controller_talk_inner', tircgo_msgs.msg.ControllerTalk, callback)
if __name__ == '__main__':
    try:
        Subscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)