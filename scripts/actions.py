#!/usr/bin/env python3

import rospy, cv2, cv_bridge
import numpy as np
import os
from sensor_msgs.msg import Image # for image subscripber
from geometry_msgs.msg import Twist, Vector3 # for cmd_vel
from sensor_msgs.msg import LaserScan # for LiDAR
import moveit_commander, math # import the moveit_commander, which allows us to control the arms

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

# perform the optimized action based on converged q learning matrix
class Actions(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node('q_learning_opitimized_actions')

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: 1, tag: 1}
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": int(x[0]), "tag": int(x[1])},
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

        # Fetch converged q_matrix. There are 64 row / states, and 9 columns (actions).
        self.q_matrix = np.loadtxt(os.path.dirname(__file__) + "/q_matrix_converged.csv", delimiter = ",")
        
        # set up ROS / OpenCV bridge
        self.bridge = cv_bridge.CvBridge()

        # initalize the debugging window
        cv2.namedWindow("window", 1)

        # a list of three opitimized action numbers
        self.opt_actions = []
        # target distance between robot and color / tag (value updated later)
        self.distance = 0.4
        # bufffer distance
        self.buffer = 0.05
        # current colored object (value update every iteration)
        self.curr_target = 1
        # current AR tag (value update every iteration)
        self.curr_tag = 2
        
        # get the optimized three action numbers based on converged q_matrix
        self.get_action()
        
        # the min distance of the robot front (front 9-degrees)
        self.front_dist = 5
        # counter of how many actions performed
        self.counter = 0
        # completion flag; true - finished
        self.finished = False
        # detection flag; 0 = color, 1 = AR tag
        self.detect = 0

        # load DICT_4X4_50
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                Image, self.image_callback)
        
        # set up cmd_vel publisher 
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        # set up scan subscriber 
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.processing_scan)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # set initial arm position 
        self.move_group_arm.go([0,-math.radians(30.0),0,0], wait=True)

        # set initial gripper opening width
        self.move_group_gripper.go([0.018,0.018], wait=True)

    # get the three optimized actions from the converged q matrix
    def get_action(self):
        # the first action is action that gives the greatest 
        #   value when q matrix is in state 0
        opt_action = self.q_matrix[0].argmax()
        
        # add the first action to the optimized action list
        self.opt_actions.append(opt_action)
        
        # initial state value should be [0,0,0], which means
        #   every thing is at the origin, because the robot 
        #   has not moved anything yet
        state = [0,0,0]
        
        for j in range(2): # we loop twice becase there are two more actions we need to find
                           #    we have already find the first action above
            # the state changes each time the robot performs an action
            state[int(self.actions[opt_action]["object"])] = self.actions[opt_action]["tag"]
            # find the state number that the state value matches to
            for i in range(len(self.states)):
                if self.states[i] == state: # find the state number
                    state_ind = i
            # find the action that gives the greatest value when q matrix is in the state state_ind
            opt_action = self.q_matrix[state_ind].argmax()
            # add the action to the optimized action list
            self.opt_actions.append(opt_action)
        
        print("optimized actions:", self.opt_actions)
    
    # move the robot arm to pick up / drop color
    def move_arm(self):
        
        # stop robot movement
        my_twist = Twist()
        my_twist.linear.x = 0
        my_twist.angular.z = 0
        self.cmd_pub.publish(my_twist)
        rospy.sleep(3)
        
        if self.detect == 0: # picking up color
            # move the arm to fetch the object
            arm_joint_goal = [
                0.0,
                math.radians(30.0),
                0.0,#math.radians(10.0),
                0.0#math.radians(-20.0)
            ]

            self.move_group_arm.go(arm_joint_goal, wait=True)

            # The above should finish once the arm has fully moved.
            # However, to prevent any residual movement,we call the following as well.
            self.move_group_arm.stop()  
            rospy.sleep(3)

            # close gripper
            self.move_group_gripper.go([0.000,0.000], wait=True)

            # The above should finish once the gripper has closed.
            # However, to prevent any residual movement,we call the following as well.
            self.move_group_gripper.stop()
            rospy.sleep(2)

            # pick up the object
            arm_joint_goal = [
                0.0,
                -math.radians(30.0),
                0.0,#math.radians(10.0),
                0.0#math.radians(-20.0)
            ]
            self.move_group_arm.go(arm_joint_goal, wait=True)

            # The above should finish once the arm has fully moved.
            # However, to prevent any residual movement,we call the following as well.
            self.move_group_arm.stop()

            rospy.sleep(3)
                
        else: # dropping the color to AR tag
            # move the object close to the ground
            arm_joint_goal = [
                0.0,
                math.radians(30.0),
                0.0,#math.radians(10.0),
                0.0#math.radians(-20.0)
            ]
            self.move_group_arm.go(arm_joint_goal, wait=True)

            # The above should finish once the arm has fully moved.
            # However, to prevent any residual movement,we call the following as well.
            self.move_group_arm.stop() 
            rospy.sleep(3)

            # open the gripper to drop the object
            self.move_group_gripper.go([0.018,0.018], wait=True)

            # The above should finish once the arm has fully moved.
            # However, to prevent any residual movement,we call the following as well.
            self.move_group_gripper.stop()
            rospy.sleep(2)

            # move up the arm after dropping
            arm_joint_goal = [
                0.0,
                -math.radians(25.0),
                0.0,#math.radians(10.0),
                0.0#math.radians(-20.0)
            ]
            self.move_group_arm.go(arm_joint_goal, wait=True)

            # The above should finish once the arm has fully moved.
            # However, to prevent any residual movement,we call the following as well.
            self.move_group_arm.stop() 
            rospy.sleep(3) 
            
            # robot steps back to prevent itself from hitting the dropped color
            my_twist = Twist()
            my_twist.linear.x = -0.1
            self.cmd_pub.publish(my_twist)
            rospy.sleep(1)
            my_twist.linear.x = 0
            self.cmd_pub.publish(my_twist)
        
        # change the flag
        self.detect = 1 - self.detect

    # identify the color / AR tag and move the robot close to the color / tag
    def image_callback(self, msg):
        
        if self.finished == True: # if finished three actions
            # do nothing
            my_twist = Twist()
            my_twist.angular.z = 0
            my_twist.linear.x = 0
            print("finished!")
        else: # three optimized actions havent been finished
            if self.detect == 0: # detecting color
                # update the current target based on the counter as an index of the optimized action lists
                self.curr_target = self.actions[self.opt_actions[self.counter]]["object"]
                # update the target distance for getting the color
                self.distance = 0.18
                # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                
                # define the upper and lower bounds for colors respectively
                if self.curr_target == 0: # pink
                    lower_color = (110, 80, 100) 
                    upper_color = (170, 255, 255) 
                elif self.curr_target == 1: # green
                    lower_color = np.array([10,80,100])
                    upper_color = np.array([50,255,255])
                else: # blue
                    lower_color = np.array([40,80,100])
                    upper_color = np.array([110,255,255])
                    
                # this erases all pixels that aren't target color
                mask = cv2.inRange(hsv, lower_color, upper_color)
                # count the number of detected colored pixels from the mask
                n_white_pix = np.sum(mask == 255)

                # image data
                h, w, d = image.shape

                # using moments() function, the center of the colored pixels is determined
                M = cv2.moments(mask)

                # initialize twist msg
                my_twist = Twist()

                # if there are any colored pixels found AND the number colored pixels hits a threshold
                if M['m00'] > 0 and n_white_pix > 150:
                    # center of the colored pixels in the image
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    
                    # a red circle is visualized in the debugging window to indicate
                    # the center point of the colored pixels
                    cv2.circle(image, (cx, cy), 10, (0,0,255), -1)

                    # based on the location of the line (approximated
                    #   by the center of the colored pixels), implement
                    #   proportional control to have the robot move towards
                    #   and face the colored object 
                    k_a = 0.5
                    k_l = -0.1
                    e_a = (w/2 - cx)/w
                    
                    if (abs(e_a) > 0.01): # if the robot is not facing the target
                        my_twist.angular.z = k_a * e_a
                    else: # the robot is roughly facing the target
                        my_twist.angular.z = 0
                    
                    if self.front_dist == 0: # the robot does not detect anything in the front
                        my_twist.linear.x = 0.1
                    else: # the robot detects the object in the front
                        dx = abs(self.distance - self.front_dist) # the distane the robot still needs to travel
                                                                  #    to hit the target distance
                        if dx < self.buffer: # if the robot hits the target distance
                            my_twist.linear.x = 0
                        else: # if the robot is far from the target distance
                            e_l = (self.distance - self.front_dist)/self.front_dist
                            my_twist.linear.x = k_l * e_l
                
                else: # the robot cant find the target object in the camera view
                    my_twist.angular.z = 0.5
                    
            else: # detecting AR tag
                # update the current tag based on the counter as an index of the optimized action lists
                self.curr_tag = self.actions[self.opt_actions[self.counter-1]]["tag"]
                # update the target distance for moving towards tag
                self.distance = 0.4
                # converts the incoming ROS message to OpenCV format and gray
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

                # image data
                h, w, d = image.shape
                
                # search for tags from DICT_4X4_50 in a GRAYSCALE image
                corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, self.aruco_dict)
                
                # initialize twist msg
                my_twist = Twist()
                
                # corners is a 4D array of shape (n, 1, 4, 2), where n is the number of tags detected
                # each entry is a set of four (x, y) pixel coordinates corresponding to the
                # location of a tag's corners in the image
                # ids is a 2D array array of shape (n, 1)
                # each entry is the id of a detected tag in the same order as in corners
                
                # we take the average x, y value of the location of the four pixel coordinates of 
                # the target AR tag as the cx cy (i.e., center of the red dot)
                tot_x = 0
                tot_y = 0
                cx = 0
                cy = 0
                tag_found = False
                if ids is not None: # if camera detects some AR tags
                    for i in range(len(ids)): # loop through all detected AR tags
                        if ids[i][0] == self.curr_tag: # if find the target AR tag
                            for j in range(4): # loop through the x,y location of the four pixel coordinates 
                                tot_x += corners[i][0][j][0] #x
                                tot_y += corners[i][0][j][1] #y
                            cx = int(tot_x / 4) # take the average x value as cx
                            cy = int(tot_y / 4) # take the average y value as cy
                            tag_found = True # update the tag_found flag
                

                # if find the target AR tag
                if tag_found:

                    # a red circle is visualized in the debugging window to indicate
                    # the center point of the target AR tag
                    cv2.circle(image, (cx, cy), 10, (0,0,255), -1)

                    # based on the location of the line (approximated
                    #   by the center of the colored pixels), implement
                    #   proportional control to have the robot move towards
                    #   and face the target AR tag 
                    k_a = 0.5
                    k_l = -0.1
                    e_a = (w/2 - cx)/w
                    if (abs(e_a) > 0.01): # if the robot is not facing the tag
                        my_twist.angular.z = k_a * e_a
                    else: # the robot is roughly facing the tag
                        my_twist.angular.z = 0
                    
                    if self.front_dist == 0: # the robot does not detect anything in the front
                        my_twist.linear.x = 0.1
                    else: # the robot detects the object in the front
                        dx = abs(self.distance - self.front_dist) # the distane the robot still needs to travel
                                                                  #    to hit the target distance
                        if dx < self.buffer: # if the robot hits the target distance
                            my_twist.linear.x = 0
                        else: # if the robot is far from the target distance
                            e_l = (self.distance - self.front_dist)/self.front_dist
                            my_twist.linear.x = k_l * e_l
                
                else: # the robot cant find the target object in the camera view
                    my_twist.angular.z = 0.5
                    
            # the robot stops in front of the target color / tag
            if (my_twist.angular.z == 0) and (my_twist.linear.x == 0):
                if (self.detect == 0): # if detecting color
                    self.counter += 1 # update the action counter
                if self.counter < 4: # the robot has not finished performing three actions
                                     #    since we update the counter after identifying the color
                                     #    and before identifying the tag, so we have to adjust
                                     #    this number to 4 instead of 3 to let the last robot
                                     #    arm action to be performed
                    self.move_arm() # grab / drop the color
                    print ("move arm")
                else: # the robot has finished performing the three optimized actions
                    # robot does nothing
                    my_twist.angular.z = 0
                    my_twist.linear.x = 0
                    self.finished = True
                    print("finished!")

        # publish the robot movement msg
        self.cmd_pub.publish(my_twist)

        # shows the debugging window with the red circle
        cv2.imshow("window", image)
        cv2.waitKey(3)

    # process the scan msg to get the min distance of the robot front (front 9 degrees)
    def processing_scan(self, data):
        self.front_dist = min(data.ranges)
        # we only consider the front 9 degrees because the robot will always adjust itself
        #    to face the target before performing any arm actions
        front_index = [0,1,2,3,4,359,358,357,356]
        min_value = 0 #initialize min value as 0
        for i in range(len(front_index)): # loop through the degrees
            if i == (len(front_index)-1): # if we loop to the last degree, we exit the loop
                break
            if data.ranges[front_index[i]] != 0: #if each value in the degree is not zero
                if data.ranges[front_index[i]] < data.ranges[front_index[i+1]]: # compare the current degree and next to find the smaller one
                    min_value = data.ranges[front_index[i]] 
                else:
                    min_value = data.ranges[front_index[i+1]] 
        self.front_dist = min_value
        

    def run(self):
        rospy.spin() # keep the robot running

if __name__ == "__main__":
    
    node = Actions()
    node.run() # keep the program running
