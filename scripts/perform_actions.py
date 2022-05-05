#!/usr/bin/env python3

import rospy, cv2, cv_bridge
from q_learning_project.msg import RobotMoveObjectToTag, RobotArmAction
import numpy as np
import send_actions
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist

class PerformActions(object):
    
    def __init__(self):

        # Initialize this node
        rospy.init_node('perform_actions')

        #initialize parameters
        self.color = None
        self.tag_id = None
        self.color_dict_HSV = {
              'pink': [[9, 255, 255], [0, 50, 70]], #or 10, 255, 255 low:0, 50, 50
              'green': [[89, 255, 255] , [36, 50, 70]], #or upper:80,255,255 low:65,60,60
              'blue': [[128, 255, 255], [90, 50, 70]],
        }

        #statuses
        self.pick_up_color = False
        self.put_down_color = False
        self.go_to_tag = False
        self.go_to_color = True
        
        #lidar scan closest obj distance
        self.closest = None

        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # initalize the debugging window
        cv2.namedWindow("window", 1)

        # Set up subscriber for robot actions
        rospy.Subscriber("/robot_action", RobotMoveObjectToTag, self.perform_action)

        # Set up publisher for action completion
        self.action_completion_pub = rospy.Publisher("/robot_action_completed", RobotMoveObjectToTag, queue_size=10)

        # Set up publisher for robot arm actions
        self.arm_action_pub = rospy.Publisher("/robot_arm_action", RobotArmAction, queue_size=10)

        # Set up publisher for robot movement
        self.move_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # Set up subscriber to robot's RGB camera  
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        # Set up subscriber for LIDAR scan
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Allow publishers/subscribers time to set up
        rospy.sleep(1)


    def scan_callback(self, data):
        closest = 5
        front = data.ranges[0:30] + data.ranges[330:360]
        if self.closest is not None:
            self.closest = min(self.closest, min(front))
        else:
            self.closest = min(front)
        
    def color_object_handler(self, img):
        self.color = "pink"
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_bound = np.array(self.color_dict_HSV[self.color][1])
        upper_bound = np.array(self.color_dict_HSV[self.color][0])
        # this erases all pixels that aren't our color
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        #need to filter search scope of img like in lab b?
        
        # using moments() function, the center of the yellow pixels is determined
        M = cv2.moments(mask)

        # if there are any specified color pixels found
        if M['m00'] > 0:
                # center of the color pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                print("cx:", cx)
                print("cy:", cy)

                # a red circle is visualized in the debugging window to indicate
                # the center point of the yellow pixels
                # hint: if you don't see a red circle, check your bounds for what is considered 'yellow'
                cv2.circle(img, (cx, cy), 20, (0,0,255), -1)

                #if close enough to color object, update status and stop
                if self.closest is not None and self.closest <= 0.2:
                    #update status
                    self.go_to_color = False
                    self.pick_up_color = True

                    move = Twist()
                    move.linear.x = 0
                    move.angular.z = 0
                    self.move_pub.publish(move)
                    return

                #proportional control 
                #move = Twist()
                #move.linear.x = 0.1
                #move.angular.z = (-cx + 145) * .003

                #print("angular:", move.angular.z)
                
                #self.move_pub.publish(move)

        # shows the debugging window
        # hint: you might want to disable this once you're able to get a red circle in the debugging window
        cv2.imshow("window", img)
        cv2.waitKey(3)

    #handle 2 statuses: go_to_color and go_to_tag
    def image_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        if self.go_to_color:
            self.color_object_handler(img)

        


    def perform_action(self, data):
        
        # Retrieve color and tag ID of object and tag associated with action
        self.color = data.robot_object
        self.tag_id = data.tag_id

        # TODO: Move to colored object (maybe sleep here so that have time for robot to go to object)

       
        # Pick up object
        self.arm_action_pub.publish("pick_up")
        print('started picking up')
        rospy.sleep(18) # change this depending on how long it takes to pick up an object, or create action completion pub
        
        
        # TODO: Move to tag

        # Put down object
        self.arm_action_pub.publish("put_down")
        print('started putting down')
        rospy.sleep(18) # change this depending on how long it takes to put down an object, or create action completion pub
        
        print('setting previous action completed')

        self.action_completion_pub.publish(data)

        
    def run(self):
        
        rospy.spin()


if __name__ == '__main__':

    perform_actions_node = PerformActions()
    perform_actions_node.run()
