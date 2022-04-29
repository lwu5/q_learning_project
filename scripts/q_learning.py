#!/usr/bin/env python3

import rospy
import numpy as np
import os
import random

from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveObjectToTag
from q_learning_project.msg import QMatrix

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { object: "pink", tag: 1}
        colors = ["pink", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"object": colors[int(x[0])], "tag": int(x[1])},
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


        # ROS
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size = 10)

        # ROS 
        self.reward_sub = rospy.Subscriber("/q_learning/reward", QLearningReward, self.get_reward)

        self.q_matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size = 10)

        self.r = 0
        self.i_num = 0
        self.q_matrix = []
        self.initialize_q_matrix()
        self.q_matrix_converged = False

        self.q_learning()
        if self.q_matrix_converged == True:
            self.save_q_matrix()

        
    # initialize q matrix
    def initialize_q_matrix(self):
        for i in range(64):
            q_matrix_row = []
            self.q_matrix.append(q_matrix_row)
            for j in range(9):
                q_matrix_row.append(0.0)
        print("initialized")
        return

    def get_reward(self, data):
        self.r = data.reward
        self.i_num = data.iteration_num


    def q_learning(self):
        print("q_learning...")
        t = 0
        diff_q = 99999999
        tolerance = 0.1
        diff_num = 0
        s = 0
        alpha = 1
        gamma = 0.8
        while diff_num < 100: #100 

            #find the indices from self.action_matrix[s] that are not -1
            valid_indices = []
            for index in range(len(self.action_matrix[s])):
                if self.action_matrix[s][index] != -1:
                    valid_indices.append(index)
                    
            next_state = random.choice(valid_indices)
            a = int(self.action_matrix[s][next_state])
            """
            For debug:
            #print("valid_indices", valid_indices)
            #print("index found or next_state", next_state)
            #print("action", self.action_matrix[s][next_state])
            """
            
            #a = random.choice(list(filter(lambda x : x != -1, self.action_matrix[s]))) #this code is wrong

            # perform / publish an action

            action_msg = RobotMoveObjectToTag()
            action_msg.robot_object = self.actions[a]["object"]
            action_msg.tag_id = int(self.actions[a]["tag"])
            
            self.action_pub.publish(action_msg)
            # receive / subscribe a reward
            rospy.sleep(1)

            curr_cell = self.q_matrix[s][a]

            print("reward: ", self.r)                                                 
            self.q_matrix[s][a] += alpha * (self.r + gamma * (max(self.q_matrix[next_state]) - self.q_matrix[s][a]))

            # publish Q-matrix to /q_learning/q_matrix
            q_matrix_msg = QMatrix()
            q_matrix_msg.q_matrix = self.q_matrix
            self.q_matrix_pub.publish(q_matrix_msg)

            t = t + 1

            if  abs(self.q_matrix[s][a] - curr_cell) < tolerance:
                diff_num += 1
            print("loop #: ", t)

        self.q_matrix_converged = True
        return        
    
    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        print("save matrix is called")
        # save to file 
        np.savetxt("../scripts/q_matrix_converged.csv", self.q_matrix, delimiter = ",")
        return
    

if __name__ == "__main__":
    node = QLearning()
    rospy.spin()
