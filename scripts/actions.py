#!/usr/bin/env python3

import rospy, cv2, cv_bridge
import numpy as np
import os
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
# import the moveit_commander, which allows us to control the arms
import moveit_commander, math

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

# get the optimized action based on converged q learning matrix
class Actions(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node('q_learning_opitimized_actions')

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
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

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                Image, self.image_callback)
        
        # TODO: set up cmd_vel publisher 
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.processing_scan)

        # # the interface to the group of joints making up the turtlebot3
        # # openmanipulator arm
        # self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # # the interface to the group of joints making up the turtlebot3
        # # openmanipulator gripper
        # self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")


        # a list of opitimized action numbers
        self.opt_actions = []
        self.distance = 0.4
        self.curr_target = 2
        self.curr_tag = 1
        self.get_action()
        self.front_dist = 5

        self.detect = 0 # 0 = color, 1 = AR tag

        # load DICT_4X4_50
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)



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
        print(self.opt_actions)
    
    def move_arm(self):
        if self.detect == 0: # picking up color
            self.move_group_arm.go([0, 0, 0, 0], wait=True)
        
        else: # dropping to AR tag
            pass
        
        self.detect = 1 - self.detect

    def image_callback(self, msg):
        counter = 0
        if self.detect == 0: # detecting color
            self.distance = 0.2
            # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
            image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            
            # TODO: define the upper and lower bounds for what should be considered 'yellow'
            # Note: see class page for hints on this
            #lower_yellow = np.array([10, 80, 80]) #TODO
            #upper_yellow = np.array([50, 240, 240]) #TODO
            

            if self.curr_target == 0:
                #pink RGB - 254, 4, 153
                lower_color = (100, 0, 80) #(100, 0, 80)
                upper_color = (255, 90, 230) #(255,100,230)
                
                
            elif self.curr_target ==1:
                #green RGB #TODO
                lower_color = np.array([0,75,110])
                upper_color = np.array([90,255,255])
            else:
                #blue RGB #TODO
                lower_color = np.array([115,90,0])
                upper_color = np.array([235,255,100])
                

            #RGB
            #Pink - 254, 4, 153
            #Green - 169, 207, 0
            #Blue - 0, 192, 228


            
            # this erases all pixels that aren't yellow
            mask = cv2.inRange(image, lower_color, upper_color)

            # this limits our search scope to only view a slice of the image near the ground
            h, w, d = image.shape
            # search_top = int(3*h/4)
            # search_bot = int(3*h/4 + 20)
            # mask[0:search_top, 0:w] = 0
            # mask[search_bot:h, 0:w] = 0

            # using moments() function, the center of the yellow pixels is determined
            M = cv2.moments(mask)

            my_twist = Twist()

            # if there are any yellow pixels found
            if M['m00'] > 0:
                # center of the yellow pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                # a red circle is visualized in the debugging window to indicate
                # the center point of the yellow pixels
                # hint: if you don't see a red circle, check your bounds for what is considered 'yellow'
                cv2.circle(image, (cx, cy), 10, (0,0,255), -1)

                # TODO: based on the location of the line (approximated
                #       by the center of the yellow pixels), implement
                #       proportional control to have the robot follow
                #       the yellow line
                k_a = 5
                k_l = -0.1
                e_a = (w/2 - cx)/w
                if (abs(e_a) > 0.01):
                    my_twist.angular.z = k_a * e_a
                else:
                    my_twist.angular.z = 0
                
                if self.front_dist == 0:
                    my_twist.linear.x = 0.1
                else:
                    dx = abs(self.distance - self.front_dist)
                    if dx < 0.1:
                        my_twist.linear.x = 0
                    else:
                        e_l = (self.distance - self.front_dist)/self.front_dist
                        print("e_l:",e_l)
                        my_twist.linear.x = k_l * e_l
                print("linear x", my_twist.linear.x )
            
            else:
                my_twist.angular.z = 0.5
        else: # detecting AR tag
            self.distance = 0.4
            image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            h, w, d = image.shape
            # search for tags from DICT_4X4_50 in a GRAYSCALE image
            corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, self.aruco_dict)
            
            my_twist = Twist()

            tot_x = 0
            tot_y = 0
            cx = 0
            cy = 0
            tag_found = False
            if ids is not None:
                for i in range(len(ids)):
                    if ids[i][0] == self.curr_tag:
                        for j in range(4):
                            tot_x += corners[i][0][j][0] #x
                            tot_y += corners[i][0][j][1] #y
                        cx = int(tot_x / 4)
                        cy = int(tot_y / 4)
                        tag_found = True
            

            # if there are any yellow pixels found
            if tag_found:

                # a red circle is visualized in the debugging window to indicate
                # the center point of the yellow pixels
                # hint: if you don't see a red circle, check your bounds for what is considered 'yellow'
                cv2.circle(image, (cx, cy), 10, (0,0,255), -1)

                # TODO: based on the location of the line (approximated
                #       by the center of the yellow pixels), implement
                #       proportional control to have the robot follow
                #       the yellow line
                k_a = 5
                k_l = -0.1
                e_a = (w/2 - cx)/w
                if (abs(e_a) > 0.01):
                    my_twist.angular.z = k_a * e_a
                else:
                    my_twist.angular.z = 0
                
                if self.front_dist == 0:
                    my_twist.linear.x = 0.1
                else:
                    dx = abs(self.distance - self.front_dist)
                    if dx < 0.1:
                        my_twist.linear.x = 0
                    else:
                        e_l = (self.distance - self.front_dist)/self.front_dist
                        print("e_l:",e_l)
                        my_twist.linear.x = k_l * e_l
                print("linear x", my_twist.linear.x )
            
            else:
                my_twist.angular.z = 0.5
            
        if (my_twist.angular.z == 0) and (my_twist.linear.x == 0):
            counter += 1
        if counter > 8:
            self.move_arm()
            counter = 0

        self.cmd_pub.publish(my_twist)

            
                                   
                    
                

        # shows the debugging window
        # hint: you might want to disable this once you're able to get a red circle in the debugging window
        cv2.imshow("window", image)
        cv2.waitKey(3)

    def processing_scan(self, data):
        front_index = [0,1,2,3,4,359,358,357,356]
        dist_sum = 0
        count = 0
        for index in front_index:
            if data.ranges[index] != 0:
                count+=1
                dist_sum += data.ranges[index]
           
            print("front_dist at index",index,":",data.ranges[index])
            
        if dist_sum == 0:
            self.front_dist = dist_sum
        else:
            dist_sum /= count
            self.front_dist = dist_sum
        
        #print("front dis at 0", data.ranges[0])
        print("front_dis", self.front_dist)
        

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    
    node = Actions()
    node.run() # keep the program running
