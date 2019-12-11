#! /usr/bin/env python

from __future__ import print_function
from enum import Enum
import rospy
import actionlib
import sys
import tircgo_controller.msg
import tircgo_msgs.msg

class Scheduler():
    def __init__(self, id = ''):
        self.clt = actionlib.SimpleActionClient(id + 'robot_scheduler_controller', tircgo_controller.msg.scheduleAction)
        self.goal = tircgo_controller.msg.scheduleGoal()
    def Wait(self):
        r = rospy.Rate(1)
        success = False
        self.clt.wait_for_server()
        self.clt.send_goal(self.goal)
        self.clt.wait_for_result()
        while not rospy.is_shutdown() and not success:
            self.clt.wait_for_server()
            self.clt.send_goal(self.goal)
            self.clt.wait_for_result()
            success = True if self.clt.get_result() != None and self.clt.get_result().res == 'done' else False
            if not success:
                r.sleep()
        rospy.loginfo('Status done')
        return
    def Work(self, *args):
        self.goal = tircgo_controller.msg.scheduleGoal(act = 76, args = args)
        rospy.loginfo('Go to R%sN%s' % (args[0], args[1]))
        self.Wait()
    def Drive(self, *args):
        self.goal = tircgo_controller.msg.scheduleGoal(act = 70, args = args)
        rospy.loginfo('Drive at %s, %s for %s' % (args[0], args[1], args[2]))
        self.Wait()
    def Delay(self, *args):
        self.goal = tircgo_controller.msg.scheduleGoal(act = 56, args = args)
        rospy.loginfo('Delay for %s cs' % (args[0]))
        self.Wait()
    def Train_begin(self, *args):
        self.goal = tircgo_controller.msg.scheduleGoal(act = 73, args = args)
        rospy.loginfo('Train on %s' % (args[0]))
        self.Wait()
    def SetNode(self, *args):
        self.goal = tircgo_controller.msg.scheduleGoal(act = 74, args = args)
        rospy.loginfo('SetNode Here')
        self.Wait()
    def Train_finish(self, *args):
        self.goal = tircgo_controller.msg.scheduleGoal(act = 75, args = args)
        rospy.loginfo('Train Finished')
        self.Wait()

def Work(r, n):
    eval('master.Work(r, n)')
def Drive(v, w, t):
    eval('master.Drive(v, w, t)')
def Delay(n):
    eval('master.Delay(n)')
def Train_begin(r):
    eval('master.Train_begin(r)')
def SetNode(*args):
    eval('master.SetNode()')
def Train_finish(*args):
    eval('master.Train_finish()')

def setup():
    return
def loop():
    return

if __name__ == '__main__':
    try:
        if len(sys.argv) == 2:
            rospy.init_node(sys.argv[1] + 'scheduler_py')
            master = Scheduler(sys.argv[1])
        else:
            rospy.init_node('scheduler_py')
            master = Scheduler()
        rospy.loginfo('Wait for controller to join')
        setup()
        while not rospy.is_shutdown():
            loop()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)