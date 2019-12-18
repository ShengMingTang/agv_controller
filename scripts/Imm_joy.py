#! /usr/bin/env python
from __future__ import print_function
from enum import Enum
import rospy
import actionlib
import sys
import tircgo_controller.msg
import tircgo_msgs.msg
import sensor_msgs.msg

class Imm_joy():
    def __init__(self, a = ''):
        self.sub = rospy.Subscriber('joy', sensor_msgs.msg.Joy, self.callback)
        self.pub = rospy.Publisher(a + 'joy', sensor_msgs.msg.Joy, queue_size=10)
    def callback(self, data):
        self.pub.publish(data)    
if __name__ == '__main__':
    try:
        print(len(sys.argv))
        if len(sys.argv) > 1:
            rospy.init_node('imm_joy', anonymous=True)
            master = []
            for id in sys.argv[1:]:
                rospy.loginfo('[Imm_joy] Build an immediate node from joy to %s' % (id))
                master.append(Imm_joy(id))
            rospy.spin()
        else:
            rospy.init_node('Dummy_imm_joy', anonymous=True)
            rospy.loginfo('Imm_joy only takes exactly one cmd arg, the name of the receiver')
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)