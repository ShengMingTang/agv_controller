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
        self.mutex = Lock()
        # controller status
        # talk
        self.talk_pub = rospy.Publisher(id + 'robot_wifi_controller_talk_inner', tircgo_msgs.msg.ControllerTalk, queue_size = 10)
        self.talk_sub = rospy.Subscriber(id + 'robot_wifi_controller_talk_outer', tircgo_msgs.msg.ControllerTalk, self.listen_callback)
        # imm
        self.imm_sub = rospy.Subscriber(id + 'to_agent', tircgo_msgs.msg.ControllerTalk, self.imm_callback)
        # channels
        self.target = -1
        self.UARThis = {}
        self.id_map = {}
        self.clt = actionlib.SimpleActionClient(id + 'robot_scheduler_controller', tircgo_controller.msg.scheduleAction)
        self.goal = tircgo_controller.msg.scheduleGoal()
    def query(self):
        opt = ''
        while opt != 'stop':
            opt = raw_input('Next cmd :')
            if opt == 'ls':
                self.mutex.acquire()
                for key in self.id_map:
                    print('%d : %s' % (key, self.id_map[key]))
                self.mutex.release()
            elif opt == 'do':
                self.mutex.acquire()
                self.target = input('targt : ')
                self.mutex.release()
            elif opt == 'preempt':
                self.mutex.acquire()
                self.target = -1
                self.mutex.release()
            elif opt != 'stop':
                print('Not recognized')
        return
    def run(self):
        inst = 0
        while self.target != -1 and rospy.is_shutdown() is False:
            if self.target == -1:
                inst = 0
                continue
            else:
                target_id = self.id_map[self.target]
                eval('self.' + self.UARThis[target_id][inst])
                inst += 1
        return
    def listen_callback(self, msg):
        tokens = msg.talk.split('\n')
        if len(tokens) != 0 and tokens[0] == 'UART_history':
            self.mutex.acquire()
            # atomic operation
            if msg.author not in self.UARThis:
                self.id_map[self.count] = msg.author
                self.count += 1
            self.UARThis[msg.author] = msg.talk
            self.mutex.release()
            # end atomic
            print('[Agent] receive a msg from ', msg.author)
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
        while not rospy.is_shutdown() and not success and not self.target != -1:
            self.clt.wait_for_server()
            self.clt.send_goal(self.goal)
            self.clt.wait_for_result()
            success = True if self.clt.get_result() != None and self.clt.get_result().res == 'done' else False
            if not success:
                r.sleep()
        if success:
            rospy.loginfo('[Agent] Status done')
        else:
            rospy.loginfo('[Agent] Prcoess preempted or failed')
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
            thread1 = Thread(target=master.query())
            thread2 = Thread(target=master.run())
            thread1.start()
            thread2.start()
            rospy.spin()
            thread1.join()
            thread2.join()
        else:
            rospy.init_node('agent_py')
            master = Agent()
            rospy.loginfo('[Agent] Wait for controller to join')
            thread1 = Thread(target=master.query())
            thread2 = Thread(target=master.run())
            thread1.start()
            thread2.start()
            rospy.spin()
            thread1.join()
            thread2.join()
    except rospy.ROSInterruptException:
        print("[Agent] program interrupted before completion", file=sys.stderr)