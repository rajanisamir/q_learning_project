#!/usr/bin/env python3

from enum import Enum
from turtle import distance
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
        self.k_p_ang_cam = 0.005
        self.k_p_ang_scan = 0.4
        self.k_p_lin = 0.2
        self.desired_obj_distance = 0.15
        self.desired_tag_distance = 0.4
        self.obj_distance_tolerance = 0.03
        self.drive_error_pixels = 30
        self.switch_to_scan = 10
        self.at_object = False
        self.angular_search_velocity = 0.5
        self.put_down_threshold = 0.08
        self.backing_up = False

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)
        
        # Allow publishers time to set up
        rospy.sleep(1)


    def process_scan(self, data):

        if self.backing_up:

            print('Backing up...')
            
            # Back up for two seconds
            self.twist.linear.x = -0.15
            self.twist.angular.z = 0
            self.move_pub.publish(self.twist)
            rospy.sleep(2)

            # Spin around
            self.twist.linear.x = 0
            self.twist.angular.z = np.pi / 2
            self.move_pub.publish(self.twist)
            rospy.sleep(2)

            self.backing_up = False

            return

        if self.goal == Goal.PICK_UP or self.goal == Goal.PUT_DOWN:

            self.twist.linear.x = 0
            self.twist.angular.z = 0

        else:

            # If the camera data indicates we're at the object, start using scan data.
            if self.at_object:

                obj_ang = data.angle_min
                self.twist.linear.x = 0
        
                if obj_ang == 0:
                    print('PICKING UP')
                    self.twist.angular.z = 0
                    self.goal = Goal.PICK_UP
                else:
                    self.twist.angular.z = self.k_p_ang_scan * obj_ang

            else:

                if self.target_center_x is None:
                    print('searching...')
                    self.twist.linear.x = 0
                    self.twist.angular.z = self.angular_search_velocity

                else:

                    target_center_x_error = self.img_center_x - self.target_center_x
                    self.twist.angular.z = self.k_p_ang_cam * target_center_x_error

                    target_distance = data.ranges[0] if data.ranges[0] != 0 else 2
                    desired_target_distance = self.desired_obj_distance if self.goal == Goal.GO_TO_OBJECT else self.desired_tag_distance
                    distance_error = target_distance - desired_target_distance

                    if abs(target_center_x_error) < self.drive_error_pixels:
                        self.twist.linear.x = self.k_p_lin * distance_error
                    else:
                        self.twist.linear.x = 0

                    if self.goal == Goal.GO_TO_OBJECT and abs(target_center_x_error) < self.switch_to_scan and abs(distance_error) < self.obj_distance_tolerance:
                        self.at_object = True
                    
                    if self.goal == Goal.GO_TO_TAG and abs(distance_error) < self.put_down_threshold:
                        print('finished going to tag')
                        self.goal = Goal.PUT_DOWN
                        self.twist.linear.x = 0
                        self.twist.angular.z = 0
                    
        # Clamp angular speed to 1.82 rad/s and linear speed to 0.26 m/s
        #   to ensure we don't exceed the maximum velocity of the Turtlebot.
        self.twist.angular.z = min(1.82, max(self.twist.angular.z, -1.82))
        self.twist.linear.x = min(0.26, max(self.twist.linear.x, -0.26))
    
        # Publish Twist message to cmd_vel.
        self.move_pub.publish(self.twist)
        
        
    def color_object_handler(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_bound = np.array(self.color_dict_HSV[self.color][0])
        upper_bound = np.array(self.color_dict_HSV[self.color][1])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        #need to filter search scope of img like in lab b?
        
        # using moments() function, the center of the yellow pixels is determined
        M = cv2.moments(mask)

        # if there are any specified color pixels found
        if M['m00'] > 10:
                # center of the color pixels in the image
                self.target_center_x = int(M['m10']/M['m00'])
                self.target_center_y = int(M['m01']/M['m00'])
                print("target_center_x:", self.target_center_x)
                print("target_center_y:", self.target_center_y)
        
        else:
            self.target_center_x = None
            self.target_center_y = None

        # shows the debugging window
        # hint: you might want to disable this once you're able to get a red circle in the debugging window
        # cv2.imshow("window", img)
        # cv2.waitKey(3)
    
    def tag_handler(self, img):

        # turn the image into a grayscale
        grayscale_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_img, aruco_dict)
        
        if ids is None:
            self.target_center_x = None
            self.target_center_y = None
            print('no tags found')
            return

        ids = np.ndarray.flatten(ids)
        tag_idx = None
        for idx, id in enumerate(ids):
            if id == self.tag_id:
                tag_idx = idx

        if tag_idx is None:
            self.target_center_x = None
            self.target_center_y = None
            print('no tag matches found')
            return
        
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

        if self.color is None or self.tag_id is None:
            print('waiting for color and/or tag ID')
            return

        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        img_width, img_height, img_depth = img.shape
        self.img_center_x = img_width / 2
        self.img_center_y = img_height / 2

        if self.goal == Goal.GO_TO_OBJECT:
            self.color_object_handler(img)

        elif self.goal == Goal.GO_TO_TAG:
            self.tag_handler(img)

    
    def perform_action(self, data):

        if data.robot_object == 'None':
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.move_pub.publish(self.twist)
            rospy.signal_shutdown('Finished')
        
        # Retrieve color and tag ID of object and tag associated with action
        self.color = data.robot_object
        self.tag_id = data.tag_id
        
        # Pick up object
        while not self.goal == Goal.PICK_UP:
            pass

        self.arm_action_pub.publish("pick_up")
        print('started picking up')
        rospy.sleep(18) # change this depending on how long it takes to pick up an object, or create action completion pub
        
        self.at_object = False # stop using scan data

        self.backing_up = True

        self.goal = Goal.GO_TO_TAG

        # Put down object
        while not self.goal == Goal.PUT_DOWN:
            pass

        self.arm_action_pub.publish("put_down")
        print('started putting down')
        rospy.sleep(18) # change this depending on how long it takes to put down an object, or create action completion pub
        
        self.backing_up = True

        self.goal = Goal.GO_TO_OBJECT

        self.action_completion_pub.publish(data)

        
    def run(self):
        
        rospy.spin()


if __name__ == '__main__':

    perform_actions_node = PerformActions()
    perform_actions_node.run()
