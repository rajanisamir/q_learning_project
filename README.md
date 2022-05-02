# q_learning_project

## Writeup

## Implementation Plan

### Objectives Description
The goal of this project is to program the robot to perceive, interact with, and organize items in its environment through a reinforcement learning model, using the Q-learning algorithm. The code should execute the Q-learning algorithm in order to generate a converged Q-matrix, and it should be able to use this matrix to guide the robot to pick up colored objects and place them in front of AR tags in order to maximize the future rewards of its actions. In doing this, the robot should be able to distinguish between and determine the positions of both the colored objects and the AR tags.

### High-Level Description
To determine which colored objects belonged in front of each AR tag, we generated a Q-matrix to track the quality of taking each available action in each possible state.

ADD HIGH-LEVEL DESCRIPTION OF HOW ALGORITHM WORKS

After learning the Q-matrix, to determine which action to take (i.e. which colored object to place in front of each AR tag), we looked up the action (column) with the highest Q-value for our state, and executed that action, which maximized the expected future reward.

### Team Members: Samir Rajani and Jason Chee

### Descriptions of Components

**Q-learning algorithm**
- Executing the Q-learning algorithm
  - We will initialize the Q matrix by setting the values of all possible actions in each state to 0 and initialize the time step to 0. We’ll set up a loop that executes until a convergence condition is met (discussed in the next point); for each iteration in the loop, we will take the row from the action matrix corresponding with our current state and select a random action that is valid (not -1). We then want to publish to the robot_action (/q_learning/robot_action) topic to execute our action. We’ll also have a callback function for the reward subscriber topic (/q_learning/reward) to get the reward from our executed action. Finally, we will update the Q value for the corresponding state according to the update formula, initially using values of α = 1 and γ = 0.8, and we’ll increment the time step.
  - To test the execution of the Q-learning algorithm, we’ll check to see if the values for each state-action pair eventually converge. We’ll update the α and γ values if necessary (e.g. to optimize the runtime). We can also log the rewards for each time step and ensure that the Q-values are changing in the way we are expecting. We will also run the phantom robot movement and the virtual reset world nodes to help debug and see how our Q matrix looks.

- Determining when the Q-matrix has converged
  - We’ll set up parameters corresponding with the convergence threshold and number of time steps (e.g. 10) before checking for convergence. We will also initialize an array to keep track of the previous Q-matrix. Every time in the Q-learning algorithm loop we have reached the specified number of time steps, we will sum the absolute values of the differences between the corresponding current values in the Q-matrix and stored previous values, and if this sum is below the threshold, the convergence criterion has been met, and we will break from the loop.
  - To test our convergence criterion, we will run the code for many more iterations after the Q-matrix has supposedly converged and check to see if the values have changed significantly. We will update the threshold parameter if the convergence criterion is either too lenient or too strict (i.e. takes too long).
- Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward
  - For each state (row) in the Q-matrix, we will find the action corresponding with the maximum Q-value. This action maximizes the expected reward for the robot in the given state.
  - We will print out the Q-matrix and ensure that the robot takes the actions in each state corresponding with the maximum Q-value for that row.

**Robot perception**
- Determining the identities and locations of the three colored objects
  - To determine locations of the colored objects, we will modify the code used for the robot line follower in Lab B. In particular, we will define HSV ranges for each color, get images from the /camera/rgb/image_raw topic, mask the image to erase pixels that are not a particular color, and compute the center of the pixels within this range. We will use the /scan topic to determine the location of each object more precisely, comparing it with the image data to match up the locations with the colors.
  - As in Lab B, we’ll output a dot in the camera feed corresponding with the location of the center of each color and verify that it is roughly located at the center of the objects we’d like to detect. We’ll also output the scan data to make sure it lines up with these locations.
- Determining the identities and locations of the three AR tags
  - To determine the locations and identities of the AR tags, we will first retrieve images from the /camera/rgb/image_raw topic. We will then convert this image to grayscale, load in the ArUco dictionary, and pass the image to the cv2.aruco.detectMarkers function to search for tags. Based on the positions of the tag’s corners, we will compute the x and y coordinates corresponding with the center of the tag. The identities will be returned directly from the detectMarkers function.
  - We will test the accuracy of the determined locations of the tags by saving an image from the camera feed, roughly estimating the pixel coordinates of the center of each tag, and comparing this with the center computed from the corner locations outputted by the detectMarkers function. For each location, we will also verify the tag IDs based on the documentation of the AR tags.

**Robot manipulation & movement**
- Picking up and putting down the colored objects with the OpenMANIPULATOR arm
  - We’ll use the Movelt ROS package to control the arm. We would want to determine what angle each of the arm and gripper joints should be, and we can use the GUI to find these values. We’ll then call the go function corresponding to our arm and gripper goals to actually move our arm and gripper. We would do this process for both picking up the object and putting it down.
  - To test, we can use the ROBOTIS GUI to help determine our joint values and simulate how our robot would pick up an object. We would then try to control the actual turtlebot’s arm and see if it can pick up/put down one of the colored objects successfully.
- Navigating to the appropriate locations to pick up and put down the colored objects
  - The locations of the colored objects are determined by the camera data and verified by the scan data; after determining which object we’d like to pick up, we will exercise proportional control on the robot, using the scan data to determine the discrepancy between the angle and distance to the object and the desired angle (0°) and distance to the object (e.g. 0.4 m). To get to the appropriate tag location to put the object down, we will exercise proportional control on the angular velocity directly from the camera data. For example, if the desired tag is on the right side of the image, we will turn the robot to the right. We will use scan data from the robot’s LiDAR to determine the distance from the tag and also exercise proportional control on the linear velocity accordingly. Alternatively, we could estimate the area of the tag using the corner positions to ensure it takes up an appropriate amount of the image frame.
  - We will test the navigation to the desired location first in Gazebo and then on the physical robots. We will vary the desired distance to the object so that the robot arm is neither too close nor too far to pick up the object. We will also adjust the proportional control constants for both linear and angular velocity.

### Timeline
_We aim to have components implemented by the following times:_

**Q-learning algorithm**
- Executing the Q-learning algorithm: Wednesday, April 27th
- Determining when the Q-matrix has converged: Thursday, April 28th
- Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward: Saturday, April 30th

**Robot perception**
- Determining the identities and locations of the three colored objects: Monday, May 2nd
- Determining the identities and locations of the three AR tags: Tuesday, May 3rd

**Robot manipulation & movement**
- Picking up and putting down the colored objects with the OpenMANIPULATOR arm: Wednesday, May 4th
- Navigating to the appropriate locations to pick up and put down the colored objects: Thursday, May 5th

We will use the remaining time to perform further tests of our implementation and to tie up any loose ends.
