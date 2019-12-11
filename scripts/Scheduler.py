#! /usr/bin/env python

from __future__ import print_function
from enum import Enum
import rospy
import actionlib
import sys
import tircgo_controller.msg
import tircgo_msgs.msg

rospy.init_node('scheduler_py')

from ScheduleKind import *
from schedule import *


if __name__ == '__main__':
    try:
        rospy.loginfo('Wait for controller to join')
        clt_.wait_for_server()
        setup()
        while not rospy.is_shutdown():
            loop()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
