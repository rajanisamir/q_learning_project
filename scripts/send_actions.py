#!/usr/bin/env python3

import rospy
from q_learning_project.msg import RobotMoveObjectToTag
import numpy as np
import os

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class SendActions(object):
    
    def __init__(self):

        # Initialize this node
        rospy.init_node('send_actions')

        # Set up publisher for moving objects to a tag
        self.action_pub = rospy.Publisher("/robot_action", RobotMoveObjectToTag, queue_size=10)

        # Allow publisher time to set up
        rospy.sleep(1)

        # Load Q-matrix from the csv file
        self.q_matrix = np.loadtxt("q_learning_project/scripts/q_matrix.csv", delimiter=',')

        # Load action matrix from txt file
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch self.actions, which is an array of dictionaries where the row index corresponds
        #   to the action number, and the value has the following form: { object: "pink", tag: 1}
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))

    # Publishes a specified action for the robot to execute
    def publish_action(self, action):
        # Pull action object and tag from passed dictionary, and then publish it
        action_object = RobotMoveObjectToTag()
        action_object.robot_object = action['object']
        action_object.tag_id = action['tag']
        self.action_pub.publish(action_object)
        
    def run(self):
        
        # initialize state to 0
        state = 0

        # find action in Q-matrix row corresponding with the current state with the largest reward
        optimal_action_idx = np.argmax(self.q_matrix[state])
        optimal_action = self.actions[optimal_action]

        # publish the optimal action
        self.publish_action(optimal_action)

if __name__ == '__main__':
    send_actions_node = SendActions()
    send_actions_node.run()
