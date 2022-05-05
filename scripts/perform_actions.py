#!/usr/bin/env python3

import rospy
from q_learning_project.msg import RobotMoveObjectToTag, RobotArmAction
import numpy as np

class PerformActions(object):
    
    def __init__(self):

        # Initialize this node
        rospy.init_node('perform_actions')

        # Set up subscriber for robot actions
        rospy.Subscriber("/robot_action", RobotMoveObjectToTag, self.perform_action)

        # Set up publisher for robot arm actions
        self.arm_action_pub = rospy.Publisher("/robot_arm_action", RobotArmAction, queue_size=10)

        # Allow publisher time to set up
        rospy.sleep(1)

    
    def perform_action(self, data):
        
        # Retrieve color and tag ID of object and tag associated with action
        color = data.robot_object
        tag_id = data.tag_id

        # TODO: Move to colored object

        # Pick up object
        self.arm_action_pub.publish("pick_up")
        rospy.sleep(10) # change this depending on how long it takes to pick up an object, or create action completion pub
        
        # TODO: Move to tag

        # Put down object
        self.arm_action_pub.publish("put_down")
        rospy.sleep(10) # change this depending on how long it takes to put down an object, or create action completion pub

        
    def run(self):
        
        rospy.spin()


if __name__ == '__main__':
    perform_actions_node = PerformActions()
    perform_actions_node.run()
