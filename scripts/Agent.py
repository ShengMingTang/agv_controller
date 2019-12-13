#! /usr/bin/env python
from __future__ import print_function
from enum import Enum
import rospy
import actionlib
import sys
import tircgo_controller.msg
import tircgo_msgs.msg
from threading import Thread, Lock

class Agent():
    def __init__(self, id = ''):
        # mutex
        self.count = 0
        self.count_mutex = Lock()
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
        if len(tokens) != 0 and tokens[0] == 'UART_history':
            # atomic operation
            self.count_mutex.acquire()
            self.count += 1
            count = self.count # record this count
            self.count_mutex.release()
            # end atomic
            print(tokens, sep='->')
            for cmd in tokens[1:]:
                # atomic
                self.count_mutex.acquire()
                thiscount = self.count
                self.count_mutex.release()
                # end atomic
                if count == thiscount:
                    eval('self.' + cmd)
                else:
                    break
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
        rospy.loginfo('[Agent] Status done')
        return
    def Delay(self, *args): # customed opcode
        self.goal = tircgo_controller.msg.scheduleGoal(act = 56, args = args)
        rospy.loginfo('[Agent] Delay for %s cs' % (args[0]))
        self.Wait()
    def Work(self, *args):
        self.goal = tircgo_controller.msg.scheduleGoal(act = 76, args = args)
        rospy.loginfo('[Agent] Go to R%sN%s' % (args[0], args[1]))
        self.Wait()
    def Drive(self, *args):
        self.goal = tircgo_controller.msg.scheduleGoal(act = 70, args = args)
        rospy.loginfo('[Agent] Drive at %s, %s for %s' % (args[0], args[1], args[2]))
        self.Wait()
    def Train_begin(self, *args):
        self.goal = tircgo_controller.msg.scheduleGoal(act = 73, args = args)
        rospy.loginfo('[Agent] Train on %s' % (args[0]))
        self.Wait()
    def SetNode(self, *args):
        self.goal = tircgo_controller.msg.scheduleGoal(act = 74, args = args)
        rospy.loginfo('[Agent] SetNode Here')
        self.Wait()
    def Train_finish(self, *args):
        self.goal = tircgo_controller.msg.scheduleGoal(act = 75, args = args)
        rospy.loginfo('[Agent] Train Finished')
        self.Wait()


if __name__ == '__main__':
    try:
        if len(sys.argv) >= 2:
            rospy.init_node(sys.argv[1] + 'agent_py')
            master = Agent(sys.argv[1])
            rospy.loginfo('[Agent] Wait for controller to join')
            rospy.spin()
        else:
            rospy.init_node('agent_py')
            master = Agent()
            rospy.loginfo('[Agent] Wait for controller to join')
            rospy.spin()
    except rospy.ROSInterruptException:
        print("[Agent] program interrupted before completion", file=sys.stderr)