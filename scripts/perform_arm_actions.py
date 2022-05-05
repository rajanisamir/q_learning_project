#!/usr/bin/env python3

import rospy
# import the moveit_commander, which allows us to control the arms
import moveit_commander

import math

# import robot arm action message
from q_learning_project.msg import RobotArmAction


class Robot(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('perform_arm_actions')

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Robot arm action subscriber
        rospy.Subscriber("/robot_arm_action", RobotArmAction, self.arm_action_received)

        self.positions = {
            "reset": 
                {
                    "arm_joint_goal": [math.radians(0.0), math.radians(0.0), math.radians(0.0), math.radians(0.0)],
                    "gripper_joint_goal": [0.0, 0.0]
                },
            "pick_up": [
                {
                    "arm_joint_goal": [math.radians(0.0), math.radians(-60.0), math.radians(0.0), math.radians(0.0)],
                    "gripper_joint_goal": [0.019, 0.019]
                },
                {
                    "arm_joint_goal": [math.radians(0.0), math.radians(20.0), math.radians(0.0), math.radians(0.0)],
                    "gripper_joint_goal": [0.019, 0.019]
                },
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
                {
                    "arm_joint_goal": [math.radians(0.0), math.radians(-70.0), math.radians(0.0), math.radians(0.0)],
                    "gripper_joint_goal": [0.019, 0.019]
                },
            ]
        }

    def arm_action_received(self, data):

        joint_goals = self.positions[data.action]

        for joint_goal in joint_goals:

            arm_joint_goal = joint_goal["arm_joint_goal"]
            gripper_joint_goal = joint_goal["gripper_joint_goal"]

            print("Sending arm joint goal: ", arm_joint_goal)
            print("Sending gripper joint goal: ", gripper_joint_goal)

            self.move_group_arm.go(arm_joint_goal, wait=True)
            self.move_group_arm.stop() # Calling ``stop()`` ensures that there is no residual movement

            self.move_group_gripper.go(gripper_joint_goal, wait=True)
            self.move_group_gripper.stop()

            rospy.sleep(4)

    def reset_arm_position(self):

        joint_goal = self.positions['reset']

        arm_joint_goal = joint_goal["arm_joint_goal"]
        gripper_joint_goal = joint_goal["gripper_joint_goal"]

        print("Sending arm joint goal: ", arm_joint_goal)
        print("Sending gripper joint goal: ", gripper_joint_goal)

        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop() # Calling ``stop()`` ensures that there is no residual movement

        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        rospy.sleep(4)

    def run(self):

        self.reset_arm_position()
        rospy.spin()

if __name__ == '__main__':

    robot = Robot()
    robot.run()