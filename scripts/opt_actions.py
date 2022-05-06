#!/usr/bin/env python3

import rospy
import numpy as np
import os

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

# get the optimized action based on converged q learning matrix
class OptActions(object):
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
        # a list of opitimized action numbers
        self.opt_actions = []

        self.get_action()

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
    
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    
    node = OptActions()
    node.run() # keep the program running
