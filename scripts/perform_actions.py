#!/usr/bin/env python3

from enum import Enum
from turtle import distance
import rospy, cv2, cv_bridge
from q_learning_project.msg import RobotMoveObjectToTag, RobotArmAction
import numpy as np
import send_actions
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3

class Goal(Enum):
    PICK_UP = 1
    GO_TO_OBJECT = 2
    GO_TO_TAG = 3
    PUT_DOWN = 4

class PerformActions(object):
    
    def __init__(self):

        # Initialize this node
        rospy.init_node('perform_actions')

        #initialize parameters
        self.color = None
        self.tag_id = None
        self.color_dict_HSV = {
            'pink': [[152, 102, 63], [163, 192, 217]],
            'green': [[35, 114, 63] , [45, 204, 217]],
            'blue': [[95, 102, 63], [105, 192, 217]],
        }

        # Initialize goal status
        self.goal = Goal.GO_TO_OBJECT
        
        #lidar scan closest obj distance
        self.closest = None

        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # initalize the debugging window
        # cv2.namedWindow("window", 1)

        # Set up subscriber for robot actions
        rospy.Subscriber("/robot_action", RobotMoveObjectToTag, self.perform_action)

        # Set up publisher for action completion
        self.action_completion_pub = rospy.Publisher("/robot_action_completed", RobotMoveObjectToTag, queue_size=10)

        # Set up publisher for robot arm actions
        self.arm_action_pub = rospy.Publisher("/robot_arm_action", RobotArmAction, queue_size=10)

        # Set up publishers for movement, RGB camera, and LiDAR scan
        self.move_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.process_image)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)

        # Set up image recognition parameters
        self.object_center_x = None
        self.object_center_y = None
        self.img_center_x = None
        self.img_center_y = None

        # Set up proportional control parameters
        self.k_p_ang_cam = 0.005
        self.k_p_ang_scan = 0.7
        self.k_p_lin = 0.05
        self.desired_obj_distance = 0.18
        self.drive_error_pixels = 30
        self.switch_to_scan = 10
        self.pickup_angle_tolerance = 0.003 # IN RADIANS (< 1 DEGREE)

        self.at_object = False
        self.angular_search_velocity = 0.8

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)
        
        # Allow publishers/subscribers time to set up
        rospy.sleep(1)

        print('finished setup')


    def process_scan(self, data):

        if self.goal == Goal.PICK_UP or self.goal == Goal.PUT_DOWN:

            self.twist.linear.x = 0
            self.twist.angular.z = 0

        else:

            # If the camera data indicates we're at the object, start using scan data.
            if self.at_object:

                obj_ang = data.angle_min
                self.twist.linear.x = 0
        
                if obj_ang < self.pickup_angle_tolerance:
                    print('PICKING UP')
                    self.twist.angular.z = 0
                    self.goal = Goal.PICK_UP
                else:
                    self.twist.angular.z = self.k_p_ang_scan * obj_ang

            else:

                if self.object_center_x is None:

                    self.twist.linear.x = 0
                    self.twist.angular.z = self.angular_search_velocity

                else:

                    object_center_x_error = self.img_center_x - self.object_center_x
                    self.twist.angular.z = self.k_p_ang_cam * object_center_x_error

                    # obj_distances = list(filter(lambda dist: dist != 0, data.ranges[-60:60]))
                    # print(obj_distances)
                    # if len(obj_distances) == 0:
                    #     obj_distance = 1
                    # else:
                    #     obj_distance = min(obj_distances)

                    obj_distance = data.ranges[0] if data.ranges[0] != 0 else 2
                    
                    print("closest object distance: ", obj_distance)
                    distance_error = obj_distance - self.desired_obj_distance

                    # TODO: Add case for going to tag.

                    print("x center error: ", object_center_x_error)
                    print("distance error: ", distance_error)

                    if self.goal == Goal.GO_TO_OBJECT and abs(object_center_x_error) < self.switch_to_scan and abs(distance_error) < 0.1:
                        print('at object')
                        self.at_object = True
                        
                    if abs(object_center_x_error) < self.drive_error_pixels:
                        self.twist.linear.x = self.k_p_lin * distance_error
                    else:
                        self.twist.linear.x = 0

        # Clamp angular speed to 1.82 rad/s and linear speed to 0.26 m/s
        #   to ensure we don't exceed the maximum velocity of the Turtlebot.
        self.twist.angular.z = min(1.82, max(self.twist.angular.z, -1.82))
        self.twist.linear.x = min(0.26, max(self.twist.linear.x, -0.26))
    
        # Publish Twist message to cmd_vel.
        self.move_pub.publish(self.twist)
        
        
    def color_object_handler(self, img):
        self.color = "blue"
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_bound = np.array(self.color_dict_HSV[self.color][0])
        upper_bound = np.array(self.color_dict_HSV[self.color][1])
        # this erases all pixels that aren't our color
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        #need to filter search scope of img like in lab b?
        
        # using moments() function, the center of the yellow pixels is determined
        M = cv2.moments(mask)

        # if there are any specified color pixels found
        if M['m00'] > 10:
                # center of the color pixels in the image
                self.object_center_x = int(M['m10']/M['m00'])
                self.object_center_y = int(M['m01']/M['m00'])
                print("object_center_x:", self.object_center_x)
                print("object_center_y:", self.object_center_y)
        
        else:
            self.object_center_x = None
            self.object_center_y = None

        img_width, img_height, img_depth = img.shape
        self.img_center_x = img_width / 2
        self.img_center_y = img_height / 2

        # shows the debugging window
        # hint: you might want to disable this once you're able to get a red circle in the debugging window
        # cv2.imshow("window", img)
        # cv2.waitKey(3)


    #handle 2 statuses: go_to_color and go_to_tag
    def process_image(self, data):

        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        if self.goal == Goal.GO_TO_OBJECT:
            self.color_object_handler(img)

        elif self.goal == Goal.GO_TO_TAG:
            # TODO
            pass

    
    def perform_action(self, data):
        
        # Retrieve color and tag ID of object and tag associated with action
        self.color = data.robot_object
        self.tag_id = data.tag_id

        
        # TODO: Move to colored object (maybe sleep here so that have time for robot to go to object)
        
        # Pick up object
        while not self.goal == Goal.PICK_UP:
            pass

        self.arm_action_pub.publish("pick_up")
        print('started picking up')
        rospy.sleep(18) # change this depending on how long it takes to pick up an object, or create action completion pub
        
        return

        self.goal = Goal.GO_TO_TAG

        # TODO: Move to tag

        self.goal = Goal.PUT_DOWN

        # Put down object
        self.arm_action_pub.publish("put_down")
        print('started putting down')
        rospy.sleep(18) # change this depending on how long it takes to put down an object, or create action completion pub
        
        self.goal = Goal.GO_TO_OBJECT

        self.action_completion_pub.publish(data)

        
    def run(self):
        
        rospy.spin()


if __name__ == '__main__':

    perform_actions_node = PerformActions()
    perform_actions_node.run()
