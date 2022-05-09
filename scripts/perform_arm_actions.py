#!/usr/bin/env python3

import rospy
import moveit_commander # Import the moveit_commander, which allows us to control the arms
import math
from q_learning_project.msg import RobotArmAction # Import robot arm action message


class PerformArmActions(object):

    # Perform initialization for performing arm actions
    def __init__(self):

        # Initialize this node
        rospy.init_node('perform_arm_actions')

        # The interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # The interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Set up subscriber to receive robot arm actions (pick_up or put_down)
        rospy.Subscriber("/robot_arm_action", RobotArmAction, self.arm_action_received)

        # Define the positions corresponding with the arm actions of "reset," "pick_up," and "put_down"
        self.positions = {
            "reset": 
                {
                    "arm_joint_goal": [math.radians(0.0), math.radians(20.0), math.radians(0.0), math.radians(0.0)],
                    "gripper_joint_goal": [0.019, 0.019]
                },
            "pick_up": [
                {
                    "arm_joint_goal": [math.radians(0.0), math.radians(20.0), math.radians(0.0), math.radians(0.0)],
                    "gripper_joint_goal": [0.0, 0.0]
                },
                {
                    "arm_joint_goal": [math.radians(0.0), math.radians(-60.0), math.radians(0.0), math.radians(0.0)],
                    "gripper_joint_goal": [0.0, 0.0]
                },
            ],
            "put_down": [
                {
                    "arm_joint_goal": [math.radians(0.0), math.radians(20.0), math.radians(0.0), math.radians(0.0)],
                    "gripper_joint_goal": [0.0, 0.0]
                },
                {
                    "arm_joint_goal": [math.radians(0.0), math.radians(20.0), math.radians(0.0), math.radians(0.0)],
                    "gripper_joint_goal": [0.019, 0.019]
                },
            ]
        }


    # Execute arm action upon receipt
    def arm_action_received(self, data):

        # Fetch joint goal positions from self.positions
        joint_goals = self.positions[data.action]

        # Move robot arm to each goal position
        for joint_goal in joint_goals:

            arm_joint_goal = joint_goal["arm_joint_goal"]
            gripper_joint_goal = joint_goal["gripper_joint_goal"]

            # Send arm joint move, and call stop() to prevent residual movement
            self.move_group_arm.go(arm_joint_goal, wait=True)
            self.move_group_arm.stop()

            # Send gripper move, and call stop() to prevent residual movement
            self.move_group_gripper.go(gripper_joint_goal, wait=True)
            self.move_group_gripper.stop()

            rospy.sleep(4)


    # Sets the arm position of the robot to point upward
    def reset_arm_position(self):

        # Fetch arm joint and gripper positions from self.positions
        joint_goal = self.positions['reset']
        arm_joint_goal = joint_goal["arm_joint_goal"]
        gripper_joint_goal = joint_goal["gripper_joint_goal"]

        # Send arm joint move, and call stop() to prevent residual movement
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop() 

        # Send gripper move, and call stop() to prevent residual movement
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        rospy.sleep(4)


    # Reset the arm position, and then wait to receive arm actions
    def run(self):

        self.reset_arm_position()
        rospy.spin()


if __name__ == '__main__':

    perform_arm_actions_node = PerformArmActions()
    perform_arm_actions_node.run()