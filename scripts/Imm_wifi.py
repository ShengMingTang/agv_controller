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
        self.sub = rospy.Subscriber(a + 'robot_wifi_controller_talk_inner', tircgo_msgs.msg.ControllerTalk, self.callback)
        self.pub = rospy.Publisher(b + 'robot_wifi_controller_talk_outer', tircgo_msgs.msg.ControllerTalk, queue_size=10)
    def callback(self, data):
        self.pub.publish(data)    
if __name__ == '__main__':
    try:
        if len(sys.argv) >= 3:
            rospy.init_node(sys.argv[1] + sys.argv[2] + 'imm_wifi', anonymous=True)
            rospy.loginfo('Build an immediate node from %s to %s' % (sys.argv[1], sys.argv[2]))
            master = Imm_wifi(sys.argv[1], sys.argv[2])
        else:
            rospy.init_node('imm_wifi', anonymous=True)
            master = Imm_wifi()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)