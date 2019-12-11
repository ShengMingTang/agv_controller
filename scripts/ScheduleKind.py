#! /usr/bin/env python
from __future__ import print_function
from enum import Enum
import rospy
import actionlib
import sys
import tircgo_controller.msg
import tircgo_msgs.msg

clt_ = actionlib.SimpleActionClient('robot_scheduler_controller', tircgo_controller.msg.scheduleAction)
goal_ = tircgo_controller.msg.scheduleGoal()

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

def Wait(goal_):
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
def Work(*args):
    goal_ = tircgo_controller.msg.scheduleGoal(act = 76, args = args)
    rospy.loginfo('Go to R%sN%s' % (args[0], args[1]))
    Wait(goal_)
def Drive(*args):
    goal_ = tircgo_controller.msg.scheduleGoal(act = 70, args = args)
    rospy.loginfo('Drive at %s, %s for %s' % (args[0], args[1], args[2]))
    Wait(goal_)
def Delay(*args):
    goal_ = tircgo_controller.msg.scheduleGoal(act = 56, args = args)
    rospy.loginfo('Delay for %s cs' % (args[0]))
    Wait(goal_)
def Train_begin(*args):
    goal_ = tircgo_controller.msg.scheduleGoal(act = 73, args = args)
    rospy.loginfo('Train on %s' % (args[0]))
    Wait(goal_)
def SetNode(*args):
    goal_ = tircgo_controller.msg.scheduleGoal(act = 74, args = args)
    rospy.loginfo('SetNode Here')
    Wait(goal_)
def Train_finish(*args):
    goal_ = tircgo_controller.msg.scheduleGoal(act = 75, args = args)
    rospy.loginfo('Train Finished')
    Wait(goal_)
