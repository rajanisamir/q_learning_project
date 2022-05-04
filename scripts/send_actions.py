#!/usr/bin/env python3

import rospy
from q_learning_project.msg import RobotArmAction
import numpy as np

class SendActions(object):
    def __init__(self):
        # Set up arm action publisher
        self.arm_action_pub = rospy.Publisher("/robot_arm_action", RobotArmAction, queue_size=10)
        rospy.sleep(1)

    def run(self):
        # Once every 20 seconds
        rate = rospy.Rate(0.05)
        actions = ["pick_up", "put_down"]
        i = 0
        while (not rospy.is_shutdown()):
            print("publishing action: ", actions[i % len(actions)])
            arm_action_msg = RobotArmAction()
            arm_action_msg.action = actions[i % len(actions)]
            self.arm_action_pub.publish(arm_action_msg)
            i += 1
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('send_actions')
    send_actions_node = SendActions()
    send_actions_node.run()
