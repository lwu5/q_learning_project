# q_learning_project

Name: Timmy Lin, Liuhao Wu

---

# Intermediate Deliverable 4/30/2022
## Objective Description
- The goal of this project is to let turtlebot learn to organize items in the environment by using reinforcement learning, specifically Q-learning algorithm. This project uses the robot's camera and LiDAR system to detect target items and moves the robot’s arm to pick up items and drop them off at the designated locations.

## High-Level Description
- We used reinforcement learning to train our Q-matrix during the process of implementing the Q-Learning Algorithm. The 2D Q-matrix is first initialized to zero. Then we keep running the following steps for each iteration till Q-matrix converges: first, randomly selecting a valid action based on current state, followed by performing the action and receiving reward from the environment; and then based on current action and state, we update our Q-matrix and switch to next state. Once the Q-matrix converges, we move from the training to the perception and control stage. (More to update on final writeup, after implementing the perception and control)

## Q-Learning Algorithm Description
1. **Selecting and executing actions for the robot (or phantom robot) to take**
- **Code Location**: Selection: Implemented with `a = int(self.action_matrix[self.s][next_state])` in function `q_learning()`. Execution: Implemented with `self.action_pub` in function `__init__` and `self.action_pub.publish(action_msg)` in function `q_learning()`.
- **Code Description**: Given the current state `s`, we first find valid indices of which values not equal to `-1` from `self.action_matrix[s]` using `s` as row and then we randomly pick one of them as our action and identify the next state based on its column. We then perform the action by publishing it to `/q_learning/robot_action` topic.

2. **Updating the Q-matrix**
- **Code Location**: Implemented with `self.q_matrix_pub` in function `__init__` and `self.q_matrix_pub.publish(q_matrix_msg)` in function `q_learning()`.
- **Code Description**: Based on robot's current action and state, next state, and reward received from the environment, we follow the Q-learing algorithm to update the Q-matrix. We choose `alpha = 1` and `gamma = 0.8` and update the Q-matrix with the formula: `alpha * (reward + gamma * (max(q_matrix[next state]) -q_matrix[current state][action]))`.


3. **Determining when to stop iterating through the Q-learning algorithm**
- **Code Location**: Implemented with `while no_change_count < 300 or t > 1000`  in function `q_learning()` where `no_change_count` is cumulated under statement `if  abs(self.q_matrix[self.s][a] - curr_cell) < tolerance`.
- **Code Description**: We set up 0.00001 as tolerance threshold. If the value change of the q_matrix is within tolerance, we will consider it as unchanged. Once the count of unchanged values exceeds `300` in the while loop, we consider the q_matrix converged. If the total iteration hits `1000`, then we consider we are not able to generate a q matrix and the q_learning training failed.

4. **Executing the path most likely to lead to receiving a reward after the Q-matrix has converged on the simulated Turtlebot3 robot**
- **Code Location**: (More to update on final writeup, after implementing the perception and control)
- **Code Description**: (More to update on final writeup, after implementing the perception and control)

---

# Implementation Plan 4/26/2022
## Implementation + Testing
1. **Q-learning algorithm**
   - Executing the Q-learning algorithm
     - **Implementation**: Create a 2-D array to store Q values and then implement the algorithm we are given in the class.
     - **Testing**: Print out some values in the matrix after the first couple iteration (i.e., t <= 2) and check them with our manual computation.
   - Determining when the Q-matrix has converged
     - **Implementation**: Create a matrix to store the last state of the Q matrix and compare that with the updated Q matrix. If the Q-matrix did not change or only change in a small and acceptable threshold for a certain amount of time (i.e., 10 times?), we can be confident that the Q-matrix has converged.
     - **Testing**: Perform additional iterations to the “converged Q matrix,” if the change of values in Q matrix is merely nothing, then we consider our implementation works.
   - Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward
     - **Implementation**: Perform the actions that give the greatest sum of rewards based on our converged Q matrix. 
     - **Testing**: Manually check if the sum of rewards in the path that the robot took is indeed the greatest compared to other possible paths.
2. **Robot perception**
   - Determining the identities and locations of the three colored objects
     - **Implementation**: Find the RGB ranges of the three colors and use the robot's camera to detect those objects. Then, move the robot to have the target color on the center of the camera view and then use the robot’s LiDAR data of range [355, 5] to check the distance between the robot and the objects.
     - **Testing**: After we choose the RGB ranges of the three colors, we will use similar code in Lab B to see if the robot is able to detect the three colors. If the robot cannot detect, we will adjust the RGB ranges of the color. We will also check if the robot adjusts its location to have the target color on the center of the camera view and print out the average of LiDAR data of range [355, 5] to check if it matches with the actual distance between robot and the target object.
   - Determining the identities and locations of the three AR tags
     - **Implementation**: We will be using ArUco library to find the AR tags in the robot’s camera image. Once the robot is able to find the target AR tag, move the robot to have the target AR tag on the center of the camera view and then use the robot’s LiDAR data of range [355, 5] to check the distance between the robot and the target AR tag.
     - **Testing**: We will check if the robot can find the AR tags and adjust its location to have the target AR tag on the center of the camera view and print out the average of LiDAR data of range [355, 5] to check if it matches with the actual distance between robot and the target AR tag.
3. **Robot manipulation & movement**
   - Picking up and putting down the colored objects with the OpenMANIPULATOR arm
     - **Implementation**: We will be familiar with programming the robot’s OpenMANIPULATOR arm, so that we can move the arm’s angle and adjust the hand to pick up and drop the colored objects.
     - **Testing**: We can test this by visually seeing if the robot’s arm can pick up and drop the objects.
   - Navigating to the appropriate locations to pick up and put down the colored objects
     - **Implementation**: Based on the LiDAR data, we move the robot to approach the target and stop the robot a certain distance from the target object so that its arm has enough space to perform the pick up and drop actions.
     - **Testing**: We can test this by visually seeing if the robot can move itself to a certain distance from the objects. We might need to test what is the best distance between the robot and the objects for the arm to perform such actions.

---

## Timeline
- Q-learning algorithm and relative writeup: 5/1
- Robot perception: 5/4
- Robot manipulation & movement: 5/8
- Final writeup: 5/9
