#! /usr/bin/env python
from __future__ import print_function
from enum import Enum
import rospy
import actionlib
import sys
import tircgo_controller.msg
import tircgo_msgs.msg
rospy.init_node('agent_py')
from ScheduleKind import *

controller_to_agent_topic = 'to_agent'
class Agent:
    def __init__(self):
        # controller status
        # talk
        self.talk_pub = rospy.Publisher('robot_wifi_controller_talk_inner', tircgo_msgs.msg.ControllerTalk, queue_size = 10)
        self.talk_sub = rospy.Subscriber('robot_wifi_controller_talk_outer', tircgo_msgs.msg.ControllerTalk, self.listen_callback)
        # imm
        self.imm_sub = rospy.Subscriber(controller_to_agent_topic, tircgo_msgs.msg.ControllerTalk, self.imm_callback)
        # channels
        self.UARThis = []
        # self.schduler = actionlib.SimpleActionClient('robot_scheduler_controller', tircgo_controller.msg.scheduleAction)
    def listen_callback(self, msg):
        tokens = msg.talk.split('\n')
        if len(tokens) == 0:
            return
        if(tokens[0] == 'UART_history'):
            for cmd in tokens[1:]:
                print(cmd)
            for cmd in tokens[1:]:
                if cmd != '':
                    eval(cmd)            
        else:
            return
    
    def imm_callback(self, msg):
        self.talk_pub.publish(msg)
        return

if __name__ == '__main__':
    try:
        tmp = Agent()
        rospy.loginfo('Wait for controller to join')
        rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)