#!/usr/bin/env python3

import rospy
import numpy as np
import os

from numpy.random import choice

from q_learning_project.msg import QMatrix, QMatrixRow, QLearningReward, RobotMoveObjectToTag
from std_msgs.msg import Header

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
            self.actions
        ))


        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the pink, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        # Set up topic names
        self.q_matrix_topic = "q_learning/q_matrix"
        self.robot_action_topic = "q_learning/robot_action"
        self.reward_topic = "q_learning/reward"

        # Set up publishers and subscribers
        self.q_matrix_pub = rospy.Publisher(self.q_matrix_topic, QMatrix, queue_size=10) # publisher for Q-matrix
        self.robot_action_pub = rospy.Publisher(self.robot_action_topic, RobotMoveObjectToTag, queue_size=10) # publisher for robot action
        self.reward_sub = rospy.Subscriber(self.reward_topic, QLearningReward, self.get_reward) # subscriber to reward topic

        rospy.sleep(5)

        print("slept")
        # Initialize parameters associated with Q-learning
        self.alpha = 1
        self.gamma = 0.8
        # self.convergence_check_steps = 100 #50
        self.convergence_check_threshold = 5000
        self.convergence_threshold = 5

        # initialize, learn, and save Q-matrix  
        self.initialize_q_matrix()
        self.learn_q_matrix()
        self.save_q_matrix()

    # initializes the Q-matrix with dimensions corresponding with the number of states and actions
    def initialize_q_matrix(self):
        self.q_matrix = np.zeros((len(self.states), len(self.actions)))
        self.publish_q_matrix()
    
    # publishes the Q-matrix
    def publish_q_matrix(self):
        q_matrix_stamped = QMatrix()
        q_matrix_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.q_matrix_topic)
        q_matrix_stamped.q_matrix = self.q_matrix.tolist()

        self.q_matrix_pub.publish(q_matrix_stamped)

    # callback function for reward subscriber
    def get_reward(self, data):
        print("Received reward: ", data.reward)
        self.curr_reward = data.reward

    # determines if self.q_matrix has converged with prev_q_matrix
    def q_matrix_has_converged(self, prev_q_matrix):
        diff = abs(self.q_matrix - prev_q_matrix)
        return np.sum(diff) < self.convergence_threshold

    # publishes action to execute
    def publish_action(self, action):
        action_stamped = RobotMoveObjectToTag()
        action_stamped.robot_object = action['object']
        action_stamped.tag_id = action['tag']
        self.robot_action_pub.publish(action_stamped)
    
    #update current state 
    def update_state(self, state, action):
        for s, a in enumerate(self.action_matrix[state]):
            if a == action:
                print("state", s)
                return s

    def get_possible_actions(self, curr_state):
        possible_actions = []
        for action in self.action_matrix[curr_state]:
            if action != -1:
                possible_actions.append(action)
        return possible_actions
            
    # executes the Q-learning algorithm
    def learn_q_matrix(self):
        t = 0
        prev_q_matrix = None
        curr_state = 0
        self.curr_reward = None
        self.total_reward = 0
        while True:
            # if sufficiently many steps have passed, check for convergence
            if self.total_reward > 5000:
                if prev_q_matrix is not None and self.q_matrix_has_converged(prev_q_matrix):
                    break
                prev_q_matrix = self.q_matrix
                self.total_reward = 0

                print("Unconverged Matrix: ", self.q_matrix)

            # choose an action at random
            possible_actions = self.get_possible_actions(curr_state)
            print("Possible Actions: ", possible_actions)
            if len(possible_actions) == 0:
                print("No Possible Actions... Resetting State")
                curr_state = 0
                possible_actions = self.get_possible_actions(curr_state)

            chosen_action = int(choice(possible_actions))
            print("Executing Action: ", chosen_action)
            self.publish_action(self.actions[chosen_action])

            updated_state = self.update_state(curr_state, chosen_action)

            while self.curr_reward is None:
                pass

            # update q_matrix
            self.q_matrix[curr_state][chosen_action] += self.alpha * (self.curr_reward + self.gamma * max(self.q_matrix[updated_state]) - self.q_matrix[curr_state][chosen_action])
            
            self.total_reward += self.curr_reward
            self.curr_reward = None
            
            print("t: ", t)
            print("Updated q: ", self.q_matrix[curr_state][chosen_action])
            # increment time step
            t += 1

            #update state
            curr_state = updated_state
            print("updated state to ", curr_state)

            # publish q-matrix
            self.publish_q_matrix()
        
        print("done")
        print(self.q_matrix)

    def save_q_matrix(self):
        np.savetxt('q_matrix.csv', self.q_matrix, fmt='%.2f', delimiter=',')

if __name__ == "__main__":
    node = QLearning()