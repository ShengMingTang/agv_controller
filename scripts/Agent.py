#! /usr/bin/env python
from __future__ import print_function
from enum import Enum
import rospy
import actionlib
import sys
import tircgo_controller.msg
import tircgo_msgs.msg


class Agent():
    def __init__(self, id = ''):
        # controller status
        # talk
        self.talk_pub = rospy.Publisher(id + 'robot_wifi_controller_talk_inner', tircgo_msgs.msg.ControllerTalk, queue_size = 10)
        self.talk_sub = rospy.Subscriber(id + 'robot_wifi_controller_talk_outer', tircgo_msgs.msg.ControllerTalk, self.listen_callback)
        # imm
        self.imm_sub = rospy.Subscriber(id + 'to_agent', tircgo_msgs.msg.ControllerTalk, self.imm_callback)
        # channels
        self.UARThis = []
        self.clt = actionlib.SimpleActionClient(id + 'robot_scheduler_controller', tircgo_controller.msg.scheduleAction)
        self.goal = tircgo_controller.msg.scheduleGoal()
        
    def listen_callback(self, msg):
        tokens = msg.talk.split('\n')
        if len(tokens) == 0:
            return
        if(tokens[0] == 'UART_history'):
            for cmd in tokens[1:]:
                print(cmd)
            for cmd in tokens[1:]:
                if cmd != '':
                    eval('self.' + cmd)
        else:
            return
    
    def imm_callback(self, msg):
        self.talk_pub.publish(msg)
        return
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


if __name__ == '__main__':
    try:
        if len(sys.argv) == 2:
            rospy.init_node(sys.argv[1] + 'agent_py')
            master = Agent(sys.argv[1])
            rospy.loginfo('Wait for controller to join')
            rospy.spin()
        else:
            rospy.init_node('agent_py')
            master = Agent()
            rospy.loginfo('Wait for controller to join')
            rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)