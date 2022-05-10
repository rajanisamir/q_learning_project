#!/usr/bin/env python3

from enum import Enum
import rospy, cv2, cv_bridge
from q_learning_project.msg import RobotMoveObjectToTag, RobotArmAction
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3

# Define the potential current goals of the robot
class Goal(Enum):
    PICK_UP = 1
    GO_TO_OBJECT = 2
    GO_TO_TAG = 3
    PUT_DOWN = 4


class PerformActions(object):
    
    # Initialize necessary components for performing actions
    def __init__(self):
        
        # Give perform_arm_actions node time to set up
        rospy.sleep(2)

        # Initialize this node
        rospy.init_node('perform_actions')

        # Set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # Set up subscriber for robot actions
        rospy.Subscriber("/robot_action", RobotMoveObjectToTag, self.perform_action)

        # Set up publisher for action completion
        self.action_completion_pub = rospy.Publisher("/robot_action_completed", RobotMoveObjectToTag, queue_size=10)

        # Set up publisher for robot arm actions
        self.arm_action_pub = rospy.Publisher("/robot_arm_action", RobotArmAction, queue_size=10)

        # Set up publisher for movement
        self.move_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Set up subscribers for RGB camera and LiDAR scan
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.process_image)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)

        # Initialize parameters corresponding with objects and tags
        self.color = None
        self.tag_id = None
        self.color_dict_HSV = {
            'pink': [[152, 102, 63], [163, 192, 217]],
            'green': [[35, 114, 63] , [45, 204, 217]],
            'blue': [[95, 102, 63], [105, 192, 217]],
        }

        # Initialize goal status as going to an object
        self.goal = Goal.GO_TO_OBJECT

        # Set up image recognition parameters
        self.target_center_x = None
        self.target_center_y = None
        self.img_center_x = None
        self.img_center_y = None

        # Set up proportional control parameters
        self.k_p_ang = 0.008
        self.k_p_ang_scan = 0.07
        self.k_p_lin = 0.3
        self.desired_obj_distance = 0.45
        self.desired_tag_distance = 0.4
        self.obj_distance_tolerance = 0.1
        self.obj_angle_tolerance = 4
        self.drive_error_pixels = 25
        self.angular_search_velocity = 0.5
        self.put_down_threshold = 0.08

        # Set up parameters corresponding with specific states of robot
        self.at_object = False
        self.backing_up = False
        self.ready_to_pick_up = False

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)
        
        # Allow publishers time to set up
        rospy.sleep(1)


    # Callback for scan topic that exercises proportional control on the robot
    def process_scan(self, data):

        # If the robot has just picked up or put down an object, move back and rotate 
        if self.backing_up:
            
            # Back up for a few seconds
            self.twist.linear.x = -0.15
            self.twist.angular.z = 0
            self.move_pub.publish(self.twist)
            rospy.sleep(3)

            # Spin around
            self.twist.linear.x = 0
            self.twist.angular.z = np.pi / 2
            self.move_pub.publish(self.twist)
            rospy.sleep(1.5)

            self.backing_up = False

            return

        # When picking up or putting down an object, stop movement
        elif self.goal == Goal.PICK_UP or self.goal == Goal.PUT_DOWN:

            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.move_pub.publish(self.twist)
            return

        # If the camera data indicates the robot is close enough to the object, stop moving
        #   and fine-tune the angle to the object using scan data
        elif self.at_object:
            
            # Get closest distance to an object in front of the robot
            min_distance = 4
            for distance in data.ranges[-15:0] + data.ranges[0:15]:
                if distance > 0:
                    min_distance = min(distance, min_distance)

            # Get angle corresponding with the closest distance to an object, and convert it to
            #   an error term for proportional control
            if min_distance != 4:
                obj_ang = data.ranges.index(min_distance)                 
                error_angle = obj_ang if obj_ang < 180 else obj_ang - 360

            # If the robot can no longer locate the object, set at_object to False
            else:
                self.at_object = False

            # Continue exercising proportional control until the angle to the object is 0
            if obj_ang != 0:                
                self.twist.angular.z = self.k_p_ang_scan * error_angle
                self.twist.linear.x = 0
                self.move_pub.publish(self.twist)

            # If the angle to the object is 0, the robot is ready to move forward and pick up the object
            else:

                self.ready_to_pick_up = True
                self.at_object = False
        
        # If the robot is facing the object precisely, move forward and pick it up
        elif self.ready_to_pick_up:

            # Move forward to the object
            self.twist.linear.x = 0.1
            self.twist.angular.z = 0
            self.move_pub.publish(self.twist)
            rospy.sleep(3.3)

            # Set the robot's goal to pick up
            self.goal = Goal.PICK_UP
            self.ready_to_pick_up = False

            return

        # If none of the above special cases apply, exercise standard proportional control
        else:

            # If the target cannot be located, spin in a circle to locate it
            if self.target_center_x is None:
                self.twist.linear.x = 0
                self.twist.angular.z = self.angular_search_velocity

            # If we can find the target, use proportional control to move toward it
            else:
                
                # Set angular velocity based on error in center position of target
                target_center_x_error = self.img_center_x - self.target_center_x
                self.twist.angular.z = self.k_p_ang * target_center_x_error
                
                # Find distance to target in front of robot
                target_distance = 4
                for distance in data.ranges[-15:0] + data.ranges[0:15]:
                    if distance > 0:
                        target_distance = min(distance, target_distance)

                desired_target_distance = self.desired_obj_distance if self.goal == Goal.GO_TO_OBJECT else self.desired_tag_distance
                distance_error = target_distance - desired_target_distance

                # Only drive forward if the error in the number of pixels from the x-position of the image
                #   center to the x-position of the target is below a certain threshold
                if abs(target_center_x_error) < self.drive_error_pixels:
                    self.twist.linear.x = self.k_p_lin * distance_error
                else:
                    self.twist.linear.x = 0

                # Mark arrival at the rough position of the object
                if self.goal == Goal.GO_TO_OBJECT and abs(target_center_x_error) < self.obj_angle_tolerance and abs(distance_error) < self.obj_distance_tolerance:
                    self.at_object = True
                    return
                
                # Set the goal to put down upon arrival at the tag
                if self.goal == Goal.GO_TO_TAG and abs(distance_error) < self.put_down_threshold:
                    self.goal = Goal.PUT_DOWN
                    self.twist.linear.x = 0
                    self.twist.angular.z = 0
                    
            # Clamp angular speed to 1.82 rad/s and linear speed to 0.26 m/s
            #   to ensure we don't exceed the maximum velocity of the Turtlebot.
            self.twist.angular.z = min(1.82, max(self.twist.angular.z, -1.82))
            self.twist.linear.x = min(0.26, max(self.twist.linear.x, -0.26))
        
            # Publish Twist message to cmd_vel.
            self.move_pub.publish(self.twist)

    
    # Compute and set center position of object target 
    def color_object_handler(self, img):

        # Convert image to HSV and mask colors that don't correspond with the current target object color
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_bound = np.array(self.color_dict_HSV[self.color][0])
        upper_bound = np.array(self.color_dict_HSV[self.color][1])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        # Filter search scope of image to inner third
        h, w, d = img.shape
        search_left = int(w / 3)
        search_right = int(2 * w / 3)
        mask[0:h, 0:search_left] = 0
        mask[0:h, search_right:w] = 0
        
        # Compute center of colored pixels
        M = cv2.moments(mask)

        # If there are more than 10 pixels found with the color of the target object, set the
        #   x- and y- positions of the target center
        if M['m00'] > 10:
            self.target_center_x = int(M['m10']/M['m00'])
            self.target_center_y = int(M['m01']/M['m00'])
        
        else:
            self.target_center_x = None
            self.target_center_y = None


    # Compute and set center position of tag target
    def tag_handler(self, img):

        # Convert image to grayscale
        grayscale_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Get target tags from the image
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_img, aruco_dict)
        
        if ids is None:
            self.target_center_x = None
            self.target_center_y = None
            return
        
        # Find tags matching the current target tag index 
        ids = np.ndarray.flatten(ids)
        tag_idx = None
        for idx, id in enumerate(ids):
            if id == self.tag_id:
                tag_idx = idx

        # Set the target center positions to None if there are no tags maching the current target index
        if tag_idx is None:
            self.target_center_x = None
            self.target_center_y = None
            return
        
        # Get average of x- and y- corner coordinates
        x_coords = []
        y_coords = []
        for coord in corners[tag_idx][0]:
            x_coords.append(coord[0])
            y_coords.append(coord[1])

        tag_center_x = sum(x_coords) / len(x_coords)
        tag_center_y = sum(y_coords) / len(y_coords)

        self.target_center_x = tag_center_x
        self.target_center_y = tag_center_y
        

    # Process image before moving to tag or object
    def process_image(self, data):
        
        # Wait for receipt of a color and tag ID
        if self.color is None or self.tag_id is None:
            return

        # Get image and set x- and y- center positions
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        img_width, img_height, img_depth = img.shape
        self.img_center_x = img_width / 2
        self.img_center_y = img_height / 2

        # Call the appopriate handler for the object or tag
        if self.goal == Goal.GO_TO_OBJECT:
            self.color_object_handler(img)

        elif self.goal == Goal.GO_TO_TAG:
            self.tag_handler(img)

    
    # Callback for actions sent by send_actions.py
    def perform_action(self, data):

        # If an empty message is received, shut down the robot
        if data.robot_object == 'None':
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.move_pub.publish(self.twist)
            rospy.signal_shutdown('Finished')
        
        # Retrieve color and tag ID of object and tag associated with action
        self.color = data.robot_object
        self.tag_id = data.tag_id
        
        # Pick up the object
        while not self.goal == Goal.PICK_UP:
            pass
        self.arm_action_pub.publish("pick_up")
        rospy.sleep(10)
        
        # Back up, and then go to the tag
        self.backing_up = True
        self.goal = Goal.GO_TO_TAG

        # Put down the object
        while not self.goal == Goal.PUT_DOWN:
            pass
        self.arm_action_pub.publish("put_down")
        rospy.sleep(10)
        
        # Back up, and then go to the next object
        self.backing_up = True
        self.goal = Goal.GO_TO_OBJECT

        # Indicate to send_actions.py that the bot has completed the requested action
        self.action_completion_pub.publish(data)

        
    # Wait to receive actions
    def run(self):
        
        rospy.spin()


if __name__ == '__main__':

    perform_actions_node = PerformActions()
    perform_actions_node.run()
