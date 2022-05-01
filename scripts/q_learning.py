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


        # ROS publisher to execute robot's actions
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag, queue_size = 10)

        # ROS subscriber to receive rewards from the environment after each action.
        self.reward_sub = rospy.Subscriber("/q_learning/reward", QLearningReward, self.get_reward)

        self.q_matrix_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size = 10)

        # initialize state to begining state (0)
        self.s = 0
        # initialize reward
        self.r = 0
        # initialize reward-receiving counter
        #   it counts how many times the robot has received rewards from the environment
        self.i_num = 0
        # initialize q matrix
        self.q_matrix = []
        # initialize flag: 
        #   if the robot receives a reward, the flag changes to True later; 
        #   otherwise, it stays False
        self.reward_change = False
        # initialize flag:
        #   turns True if the q_matrix converges (the value of q_matrix hasn't 
        #   `changed` for no_change_count times)
        self.q_matrix_converged = False
        # call to initialize q matrix
        self.initialize_q_matrix()
        # run q learning algorithm
        self.q_learning()

        if self.q_matrix_converged == True: # if q_matrix has converged
            self.save_q_matrix() # outputs to cvs
        else: # q_matrix can't converge after q_learning()
            print("q_learning failed.")

        
    # initialize q matrix
    def initialize_q_matrix(self):
        # initialize 2D matrix where the value of each cell sets to 0
        for i in range(64):
            q_matrix_row = []
            self.q_matrix.append(q_matrix_row)
            for j in range(9):
                q_matrix_row.append(0.0)
        print("initialized")

    # function for reward subscriber: get reward from the environment
    def get_reward(self, data):
        self.r = data.reward
        if ((data.iteration_num - self.i_num) != 0): # if the robot receives reward
            self.reward_change = True
        self.i_num = data.iteration_num

    # run q learning alogrithm
    def q_learning(self):
        print("q_learning...")
        # iteration counter
        t = 0
        # be low what number, we consider the matrix does not change
        tolerance = 0.00001
        # counter for number of times that q matrix does not `change`
        no_change_count = 0
        # initialize alpha and gamma for q algorithm
        alpha = 1
        gamma = 0.8


        while no_change_count < 100 or t > 1000: # while q matrix has not converged 
                                                 # we consider q matrix converged if it has not 
                                                 #  `change` after this amount of iteration
            
            #find the indices from self.action_matrix[s] that are not -1
            valid_indices = []
            for index in range(len(self.action_matrix[self.s])):
                if self.action_matrix[self.s][index] != -1: # if the action is valid
                    valid_indices.append(index)

            # randomly pick a valid action and find the next state        
            next_state = random.choice(valid_indices)
            a = int(self.action_matrix[self.s][next_state])

            # perform / publish an action
            action_msg = RobotMoveObjectToTag()
            action_msg.robot_object = self.actions[a]["object"]
            action_msg.tag_id = int(self.actions[a]["tag"])
            self.action_pub.publish(action_msg)

            # receive / subscribe a reward
            print("smth")
            while (self.reward_change == False): # while we not receive a reward
                rospy.sleep(1) # give the subscriber more time to get a reward

            self.reward_change = False # set the reward flag to False for next action

            # the value in the matrix based on current state and action
            curr_cell = self.q_matrix[self.s][a]
            
            print("reward: ", self.r)  
            # update current cell                                               
            self.q_matrix[self.s][a] += alpha * (self.r + gamma * (max(self.q_matrix[next_state]) - self.q_matrix[self.s][a]))

            # publish Q-matrix to /q_learning/q_matrix
            q_matrix_msg = QMatrix()
            q_matrix_msg.q_matrix = self.q_matrix
            self.q_matrix_pub.publish(q_matrix_msg)

            if  abs(self.q_matrix[self.s][a] - curr_cell) < tolerance: # if q_matrix is unchange
                                                                       # we consider q_matrix unchange
                                                                       #    if the value change is within tolerance
                no_change_count += 1
            print("loop #: ", t)

            # update state
            if t % 3 == 0: # no more dumbbells waiting to be moved
                self.s = 0 # back to begining state
            else: # switch state
                self.s = next_state

            # iteraction counter
            t = t + 1

        if (t > 1000): # after this number of iterations,
                       #    if the q matrix still has not converge
                       #    we consider q matrix converge failed
            self.q_matrix_converged = False
        else: # q matrix has converged
            self.q_matrix_converged = True      
    
    # save q_matrix to a file once it is done
    def save_q_matrix(self):
        np.savetxt("../scripts/q_matrix_converged.csv", self.q_matrix, delimiter = ",")
        print("q_matrix saved")
    

if __name__ == "__main__":
    node = QLearning()
    rospy.spin() # keep the program running
