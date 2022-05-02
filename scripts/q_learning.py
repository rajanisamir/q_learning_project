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

        # Set up publishers for Q-matrix and robot actions and subscriber for rewards
        self.q_matrix_pub = rospy.Publisher(self.q_matrix_topic, QMatrix, queue_size=10)
        self.robot_action_pub = rospy.Publisher(self.robot_action_topic, RobotMoveObjectToTag, queue_size=10)
        self.reward_sub = rospy.Subscriber(self.reward_topic, QLearningReward, self.get_reward)

        # Give publisher enough time to set up
        rospy.sleep(1)

        # Initialize parameters associated with Q-learning:
            # alpha: a value in (0, 1] representing the learning rate of the Q-learning algorithm
            # gamma: a value in [0, 1) representing the discount factor for future rewards in the Q-learning algorithm
            # convergence_check_threshold: the sum of rewards after which the Q-learning algorithm will check the
            #   Q-matrix for convergence
            # convergence_threshold: the sum of the element-wise absolute differences between the current Q-matrix and
            #   the Q-matrix at the previous convergence check below which the Q-matrix will be considered to have converged
        self.alpha = 1
        self.gamma = 0.8
        self.convergence_check_threshold = 10000
        self.convergence_threshold = 5

        # Initialize, learn, and save Q-matrix  
        self.initialize_q_matrix()
        self.learn_q_matrix()
        self.save_q_matrix()

    # initializes the Q-matrix with dimensions corresponding with the number of states and actions
    def initialize_q_matrix(self):
        self.q_matrix = np.zeros((len(self.states), len(self.actions)))
        self.publish_q_matrix()
    
    # publishes the current Q-matrix
    def publish_q_matrix(self):
        # add header to Q-matrix containing a timestamp and ID
        q_matrix_stamped = QMatrix()
        q_matrix_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.q_matrix_topic)
        q_matrix_stamped.q_matrix = self.q_matrix.tolist()

        self.q_matrix_pub.publish(q_matrix_stamped)

    # Callback function for the reward subscriber, which sets self.current_reward to the received reward
    def get_reward(self, data):
        self.curr_reward = data.reward

    # Determines if self.q_matrix has converged with prev_q_matrix
    def q_matrix_has_converged(self, prev_q_matrix):
        # Computes the sum of the element-wise absolute difference between current Q-matrix and previous Q-matrix;
        #   return if the sum is below the convergence threshold
        diff = abs(self.q_matrix - prev_q_matrix)
        return np.sum(diff) < self.convergence_threshold

    # Publishes a specified action for the robot to execute
    def publish_action(self, action):
        action_stamped = RobotMoveObjectToTag()
        action_stamped.robot_object = action['object']
        action_stamped.tag_id = action['tag']
        self.robot_action_pub.publish(action_stamped)
    
    # Returns the state associated with taking a specified action from a specified state
    def get_state(self, state, action):
        for s, a in enumerate(self.action_matrix[state]):
            if a == action:
                return s

    # Returns a list of actions possible from a specified state
    def get_possible_actions(self, state):
        possible_actions = []
        for action in self.action_matrix[state]:
            if action != -1:
                possible_actions.append(action)
        return possible_actions
            
    # Executes the Q-learning algorithm
    def learn_q_matrix(self):
        t = 0
        prev_q_matrix = None
        curr_state = 0
        curr_reward_sum = 0
        self.curr_reward = None

        while True:
            # If the sum of the rewards received since the last convergence check exceeds a threshold,
            #   check for convergence.
            if curr_reward_sum > self.convergence_check_threshold:
                if prev_q_matrix is not None and self.q_matrix_has_converged(prev_q_matrix):
                    break
                prev_q_matrix = self.q_matrix
                curr_reward_sum = 0

            # Choose and publish an action at random; reset the state to 0 if there are none available.
            possible_actions = self.get_possible_actions(curr_state)
            if len(possible_actions) == 0:
                curr_state = 0
                possible_actions = self.get_possible_actions(curr_state)
            chosen_action = int(choice(possible_actions))
            self.publish_action(self.actions[chosen_action])

            updated_state = self.get_state(curr_state, chosen_action)

            while self.curr_reward is None:
                pass

            # Update Q-matrix using the formula
            self.q_matrix[curr_state][chosen_action] += self.alpha * (self.curr_reward + \
                self.gamma * max(self.q_matrix[updated_state]) - self.q_matrix[curr_state][chosen_action])
            
            curr_reward_sum += self.curr_reward
            self.curr_reward = None
            
            # Increment time step and update robot's state
            t += 1
            curr_state = updated_state

            # Publish Q-matrix
            self.publish_q_matrix()
        
    def save_q_matrix(self):
        np.savetxt('q_matrix.csv', self.q_matrix, fmt='%.2f', delimiter=',')

if __name__ == "__main__":
    node = QLearning()