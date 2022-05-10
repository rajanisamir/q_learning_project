# q_learning_project

## Writeup

### Objectives Description
The goal of this project is to program the robot to perceive, interact with, and organize items in its environment through a reinforcement learning model, using the Q-learning algorithm. The code should execute the Q-learning algorithm in order to generate a converged Q-matrix, and it should be able to use this matrix to guide the robot to pick up colored objects and place them in front of AR tags in order to maximize the future rewards of its actions. In doing this, the robot should be able to distinguish between and determine the positions of both the colored objects and the AR tags.

### High-Level Description
To determine which colored objects belonged in front of each AR tag, we generated a Q-matrix to track the quality of taking each available action in each possible state. We first initialized a Q-matrix in which the rows corresponded with robot states and columns corresponded with actions. We then followed the steps outlined in the Q-learning algorithm, initializing a time step and current state to 0, and taking actions (on the simulated robot) within a loop until the Q-matrix converged. From the current state, an action is executed, and the received reward is used to update the cell in the Q-matrix corresponding with the current state and the executed action using the Q-learning formula. Once a specified amount of rewards have been received, we check if our Q-matrix has converged by taking the sum of the element-wise absolute differences between the current Q-matrix and the Q-matrix at the previous convergence check and comparing it to a threshold. Once the Q-matrix has converged, iterations are stopped. After learning the Q-matrix, to determine which action to take (i.e. which colored object to place in front of each AR tag), we looked up the action (column) with the highest Q-value for our state, and executed that action, which maximized the expected future reward.

### Q-Learning Algorithm Description
- Selecting and executing actions for the robot (or phantom robot) to take
> The code for this component is located in `q_matrix.py`, in the `learn_q_matrix` function, which in turn calls the functions `get_possible_actions`, `publish_action`, and `get_state`.

> For each iteration of the loop in `learn_q_matrix`, we call a helper function `get_possible_actions`, which returns a list of possible actions from the current state based on the provided `action_matrix`. If there are no possible actions, we reset the state to 0 and refresh the possible actions. We then randomly select an action from the list using `numpy.random.choice`, and we publish the action to the robot using `self.publish_action`, a helper function that initializes and publishes an appropriate `RobotMoveObjectToTag` message to the `\q_learning\robot_action` topic, constructing the message from a passed action dictionary parameter. We get this parameter by retrieving the value of `self.actions`, a list of such dictionaries, at the index corresponding with the action we are taking. Finally, we update the state by calling `get_state`, a helper function that retrieves the state associated with taking a specified action from a specified state by iterating through the state's row in the action matrix and returning the index for which the action matches the passed action.

- Updating the Q-matrix
> The code for this component is located in `q_matrix.py`, in the `learn_q_matrix` function, and uses the callback `get_reward`, which is registered to the reward subscriber.

> First, we wait to receive the reward from our executed action by repeatedly checking to see if `self.curr_reward` has been populated. This property gets set by the `get_reward` callback, which is registered to the reward subscriber, and simply sets `self.curr_reward` to the received reward. Then, we update the Q-matrix using the update formula, which increments the matrix element corresponding with the current state and action by the sum of the received reward and the discount-scaled difference between the maximum quality across all actions in the new state and the quality for the current action and state, all scaled by the learning rate. The discount factor gamma and learning rate alpha are set to 0.8 and 1, respectively, in the constructor for the `QLearning` class. Finally, we set `self.curr_reward` to `None` so that the next iteration can wait for it to be populated.

- Determining when to stop iterating through the Q-learning algorithm
> The code for this component is located in `q_matrix.py`, at the beginning of the loop in the `learn_q_matrix` function, which also uses the helper function `q_matrix_has_converged`.

> In each iteration of the loop in `learn_q_matrix`, we increment `curr_reward_sum`, the sum of the rewards received since the last convergence check. At the beginning of each loop iteration, if `curr_reward_sum` exceeds a threshold specified in the constructor for the `QLearning` class, which we set to 10,000, we check for convergence. First, we check if there has been a previous convergence check; if there has not, we set `prev_q_matrix` to the current Q-matrix, reset the reward sum, and wait for the next convergence check. If there has been a previous convergence check, `prev_q_matrix` will have been set to the Q-matrix at that time. In this case, we call the `q_matrix_has_converged` helper function, passing in `prev_q_matrix`. This function computes the sum of the element-wise absolute difference between current Q-matrix and previous Q-matrix and returns a Boolean value corresponding with whether the sum is below the convergence threshold specified in the constructor for the `QLearning` class, which we set to 5. If the Q-matrix has converged, we stop iterating; otherwise, we set `prev_q_matrix` to the current Q-matrix, reset the reward sum, and wait for the next convergence check.

- Executing the path most likely to lead to receiving a reward after the Q-matrix has converged on the simulated Turtlebot3 robot
> The code for this component is located in `send_actions.py`, in the `run()` function.

> We initialize the robot's state to 0 and continually loop to send actions (consisting of moving an object to a tag) to the robot while there are positive values in the Q-matrix in the row of the current state. For each iteration of the loop, we find the action in the Q-matrix row with the largest quality value, and then we use the provided `self.actions` dictionary to look up the color and tag ID associated with this action. After publishing our action, we find the robot's new state by searching the current state's row in the action matrix. 

### Robot perception Description
- Identifying the locations and identities of each of the colored objects
  - We used logic similar to lab B line follower to identify the colored objects. We subscribe to the camera/rgb/image_raw topic and determine the hsv lower and upper bound values for the colors pink, blue, and green similar to the objects' colors. We defined these values in a dictionary self.color_dict_HSV. We then erase all the pixels that aren't our specified color and determine the center of our color pixels. We implement this logic in our function color_object_handler(self, img) and call this function in our camera/rgb/image_raw subscriber callback: process_image(self, data). In color_object_handler(), we first get our lower and upper hsv values from our specified color and use the moments() function to get the center of our color pixels. We set our values self.target_center_x and self.target_center_y, which will be used for proportional control to drive to the colored object, if we get find more than 10 pixels of our color.

- Identifying the locations and identities of each of the AR tags
  - We used similar logic from the cat recognizer exercise in class. We again use the subscriber to the camera/rgb/image_raw topic to determine what tags are detected from the camera. We implement our logic in the tag_handler(self, img) function and call this in our camera/rgb/image_raw subscriber callback: process_image(self, data). Note, we have a variable called self.goal which would determine if we are heading towards the colored object or tag and therefore call either color_object_handler() or tag_handler(). In tag_handler, we turn the image into a grayscale and use the detectMarkers function to get the corners, ids, rejected_points of the tags detected. We go through the ids and get the index of our desired tag id. We then use that index to get the corners of our desired tag id and average those x and y corner coordinates to get its center. We put the center coordinates in self.target_center_x and self.target_center_y which again would be used in proprotional control to move the bot to the tag. 

### Robot manipulation and movement

- Moving to the right spot in order to pick up a colored object
  - TODO
  
- Picking up the colored object
  - TODO

- Moving to the desired destination (AR tag) with the colored object
  - We'll use proportional control on the angular and linear velocities of the turtlebot to move it towards the tag. We implement this logic in process_scan() callback function (the last else statement line 178). From our image callback function, we should have a value for self.target_center_x, which we would use to get the error term by getting the difference between it and the image center (which we also get from the image callback). Our angular velocity then becomes multiplying this error term (self.target_center_x_error) by a kp constant we define. We then use the LIDAR scan to see what's the measured distance of the tag and get the difference between that and our desired distance. We again multiply this error term (distance_error) by another kp constant we define. We set the linear velocity to this proportional control when we're within pixel range of the tag, otherwise it would beset to 0. Then we have another conditional to check once we're close enough to the tag, we stop the bot and signal that it's time to drop the object.

- Putting the colored object back down at the desired destination
  - When the turtlebot is moving towards the tag, the arm is positioned upwards so when the bot gets close enough to the tag, we then lower the arm and open the gripper to release the object. We implement this logic in the second half of the perform_action callback function for the /robot_action subscriber (perform_actions.py script) and the arm_action_received() callback function for the /robot_arm_action subscriber (perform_arm_actions.py script). In perform_action function, we publish to the /robot_arm_action topic that indicates to put down the object. This is then received in arm_action_received(), which will then use the joint goals stated for self.positions["put down"]. We got these joint goals via testing with rviz and the bot itself. 

### Challenges
A challenge we faced was when moving the turtlebot towards the colored object and then positioning it correctly to pick it up (defintely spent most of the project time on this). When testing, the turtlebot got to the colored object correctly, but when it tried to pick it up, the object was a little to the left so the arm sometimes got stuck and couldn't pick it up. We also initially had the arm go up and then come down on the object to try to pick it up. We found this first method to be very inconsistent in its success rates so we changed our method to similar to how the gif in the project page had it. We now had the arm to be lower and the bot would essentially drive straight towards the object, close the gripper, then bring the arm up. When we got a certain distance close enough to the object, we adjust it's positioning so the object is in front of the bot. Then the bot would move forward for a couple of seconds and grab the object. Some other things we changed was that is that we filtered the search scope of the image so that it only looks in the middle of the image. We also switched the proportional control to use the LIDAR scan when close enough to the object to position itself so that the object was directly in front of the bot to pick up. 
Another challenge was how to determine when a previous action was completed as we had one script that sends the optimal actions and one that would perform those actions. We decided to set up an action_completion_pub publisher in our perform_actions script and the subscriber in our send_actions script. Essentially, once we have completed an action, we would publish in our action_completion_pub and then the subscriber callback in send_actions script would know that an action was just completed, so then it could send the next action. 

### Future work
One way in which our work could be extended is by generalizing our setup to one in which the objects and tags are not lined up neatly. This could be achieved by using SLAM to allow to robot to systematically search its environment when looking for the objects and tags, rather than using our current implementation, in which the robot spins in a circle to find the objects and tags. We might also generalize the "pick up" action to work on objects of arbitrary height by using the vertical position of the object's center (found using the Raspberry Pi camera) to determine how far the robot's arm should extend downward to pick up the object. We could further extend the robot's functionality by using image classification to allow the robot to recognize arbitrary objects, training a machine learning model on images that we might want to recognize. Finally, we could take a more systematic approach in learning the Q-matrix by checking for convergence only after each action has been taken once, rather than after a fixed reward sum has been reached. This would ensure that the convergence check passes not just because a representative sample of actions has not been taken, but because the quality values of each action in each state have truly converged.

### Takeaways 
1. This project gave us a lot of freedom in organization and structuring of our code. Unlike in the particle filter project where I felt like we were essentially just filling in functions, here we were writing our own scripts and nodes and making sure everything connected with each other. Furthermore, we had to handle many different components of the bot like the arm, camera, LIDAR scan, movement. It really felt like we were pulling everything we learned into this project as we used implementation logic from many previous exercises like the line follower and cat recognizer with the proportional control. 

### Video

https://user-images.githubusercontent.com/38731359/167584471-456732c8-7935-40db-b3df-72da4e01137a.mov

## Implementation Plan

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
