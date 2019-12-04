#! /usr/bin/env python

from __future__ import print_function
from enum import Enum
import rospy
import actionlib
import sys
import tircgo_controller.msg

class OPCODE(Enum):
    NONE = 48 
    RT_UP = 49
    RT_DOWN = 50
    ND_UP = 51
    ND_DOWN = 52
    DISPLAY = 53
    AUTO_BEGIN = 54
    AUTO_FINISH = 55
    DELAY = 56
    # hardware control
    SHUTDOWN = 69
    DRIVE = 70
    SIGNAL = 71  # for LED and beeper
    CALIB = 72
    TRAIN_BEGIN = 73
    SETNODE = 74
    TRAIN_FINISH = 75
    WORK_BEGIN = 76
    WORK_FINISH = 77

rospy.init_node('scheduler_py')
clt_ = actionlib.SimpleActionClient('robot_scheduler_controller', tircgo_controller.msg.scheduleAction)
goal_ = tircgo_controller.msg.scheduleGoal()
def Wait(goal_):
    global clt_
    r = rospy.Rate(1)
    success = False
    clt_.wait_for_server()
    clt_.send_goal(goal_)
    clt_.wait_for_result()
    while not rospy.is_shutdown() and not success:
        clt_.wait_for_server()
        clt_.send_goal(goal_)
        clt_.wait_for_result()
        success = True if clt_.get_result() != None and clt_.get_result().res == 'done' else False
        if not success:
            r.sleep()
    rospy.loginfo('Status done')
    return
def Go(r, n):
    goal_ = tircgo_controller.msg.scheduleGoal(act = 76, args = (r, n))
    rospy.loginfo('Go to R%dN%d' % (r, n))
    Wait(goal_)
def Delay(n):
    goal_ = tircgo_controller.msg.scheduleGoal(act = 56, args = (n))
    rospy.loginfo('Delay for %f' % (n / 10.0))
    Wait(goal_)

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
