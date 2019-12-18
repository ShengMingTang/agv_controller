#! /usr/bin/env python
from __future__ import print_function
from enum import Enum
import rospy
import actionlib
import sys
import tircgo_controller.msg
import tircgo_msgs.msg

class Imm_wifi():
    def __init__(self, a = '', b = ''):
        self.sub1 = rospy.Subscriber(a + 'robot_wifi_controller_talk_inner', tircgo_msgs.msg.ControllerTalk, self.callback1)
        self.pub1 = rospy.Publisher(b + 'robot_wifi_controller_talk_outer', tircgo_msgs.msg.ControllerTalk, queue_size=10)
        self.sub2 = rospy.Subscriber(b + 'robot_wifi_controller_talk_inner', tircgo_msgs.msg.ControllerTalk, self.callback2)
        self.pub2 = rospy.Publisher(a + 'robot_wifi_controller_talk_outer', tircgo_msgs.msg.ControllerTalk, queue_size=10)
    def callback1(self, data):
        self.pub1.publish(data)
    def callback2(self, data):
        self.pub2.publish(data)
if __name__ == '__main__':
    try:
        if len(sys.argv) >= 3:
            rospy.init_node(sys.argv[1] + sys.argv[2] + 'imm_wifi', anonymous=True)
            rospy.loginfo('Build an immediate node from %s to %s' % (sys.argv[1], sys.argv[2]))
            rospy.loginfo('Build an immediate node from %s to %s' % (sys.argv[2], sys.argv[1]))
            master = Imm_wifi(sys.argv[1], sys.argv[2])
        else:
            rospy.init_node('imm_wifi', anonymous=True)
            rospy.loginfo('Imm_wifi takes exactly 2 arguments')
            master = Imm_wifi()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)