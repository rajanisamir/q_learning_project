#!/usr/bin/env python3

import rospy
from q_learning_project.msg import RobotMoveObjectToTag
import numpy as np
import os

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class SendActions(object):
    
    def __init__(self):

        # Give perform_arm_actions node and perform_actions node time to set up
        rospy.sleep(3)

        # Initialize this node
        rospy.init_node('send_actions')

        # Set up publisher for robot actions, consisting of moving objects to tags
        self.action_pub = rospy.Publisher("/robot_action", RobotMoveObjectToTag, queue_size=10)

        # Set up subscriber to receive when actions are completed
        self.action_completion_sub = rospy.Subscriber("/robot_action_completed", RobotMoveObjectToTag, self.set_prev_action_completed)

        # Allow publisher time to set up
        rospy.sleep(1)

        # Load Q-matrix from the csv file
        self.q_matrix = np.loadtxt(path_prefix + "q_matrix.csv", delimiter=',')

        # Load action matrix from txt file
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Track whether the previous action was completed
        self.prev_action_completed = True

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


    # Returns the state associated with taking a specified action from a specified state
    def get_state(self, state, action):

        # Search for the action in the row of action matrix corresponding with the specified initial state; return its index
        for s, a in enumerate(self.action_matrix[state]):
            if a == action:
                return s


    # Marks the previous action as completed, allowing for the next action to be published
    def set_prev_action_completed(self, data):
        self.prev_action_completed = True


    def run(self):
        
        # Initialize state to 0
        state = 0

        # Send actions while there are positive values in the Q-matrix in the row of the current state
        while np.max(self.q_matrix[state]) > 0:

            # Wait for previous action to be completed
            while not self.prev_action_completed:
                pass

            # Find action in Q-matrix row corresponding with the current state with the largest quality
            optimal_action_idx = np.argmax(self.q_matrix[state])
            optimal_action = self.actions[optimal_action_idx]

            # Publish the optimal action
            self.prev_action_completed = False
            self.publish_action(optimal_action)
            
            # Set new state based on action taken
            state = self.get_state(state, optimal_action_idx)
        
        self.publish_action({'object': 'None', 'tag': 0})

        
if __name__ == '__main__':
    send_actions_node = SendActions()
    send_actions_node.run()
