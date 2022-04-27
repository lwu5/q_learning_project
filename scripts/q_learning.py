#!/usr/bin/env python3

import rospy
import numpy as np
import os
import random

from q_learning_project.msg import QLearningReward
from q_learning_project.msg import RobotMoveObjectToTag

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

        self.q_matrix = np.matrix([64][9])

        # initialize q matrix
        for i in range(64):
            for j in range(9):
                self.q_matrix[i][j]  = 0.0

        self.q_matrix_converged = False
    
    def q_learning(self):
        t = 0
        diff_q = 99999999
        tolerance = 0.1
        diff_num = 0
        s = 0
        alpha = 1
        gamma = 0.8
        while diff_num < 20:
            a = random.choice(list(filter(lambda x : x != -1, self.action_matrix[s])))
            # perform / publish an action
            object = self.actions[a]["object"]
            tag = self.actions[a]["tag"]
            self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveObjectToTag(robot_object = object, tag_id = tag), queue_size = 10)
            # receive / subscribe a reward
            self.reward_sub = rospy.Subscriber("/q_learning/reward", QLearningReward, self.get_reward)
            curr_cell = self.q_matrix[s][a]
            self.q_matrix[s][a] += alpha * (self.r + gamma * (max(self.q_matrix[s+1])- self.q_matrix[s][a]))
            t = t + 1

            if  abs(self.q_matrix[s][a] - curr_cell) < tolerance:
                diff_num += 1
        self.q_matrix_converged = True
        return
    
    def get_reward(self, data):
        self.r = data.reward
        self.i_num = data.iteration_num
        return
        
    
    def save_q_matrix(self):
        # TODO: You'll want to save your q_matrix to a file once it is done to
        # avoid retraining
        if self.q_matrix_converged == True:
            # save to file
            np.savetxt("q_matrix_converged.csv", self.q_matrix, delimiter = ",")
        else:
            self.q_learning()
        return

if __name__ == "__main__":
    node = QLearning()
