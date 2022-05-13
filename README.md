# q_learning_project

Name: Timmy Lin, Liuhao Wu

---

# Behavior (Gif)
![q_learning](https://user-images.githubusercontent.com/66953378/168196391-a060249e-4e66-4fd8-bd39-7492959cad5c.GIF)

---

# \[UPDATED\] Final Writeup 5/11/2022
# Intermediate Deliverable 4/30/2022
## Objective Description
- The goal of this project is to let turtlebot learn to organize items in the environment by using reinforcement learning, specifically Q-learning algorithm. This project uses the robot's camera and LiDAR system to detect target items and moves the robot’s arm to pick up items and drop them off at the designated locations.

## High-Level Description
- We used reinforcement learning to train our Q-matrix during the process of implementing the Q-Learning Algorithm. The 2D Q-matrix is first initialized to zero. Then we keep running the following steps for each iteration till Q-matrix converges: first, randomly selecting a valid action based on current state, followed by performing the action and receiving reward from the environment; and then based on current action and state, we update our Q-matrix and switch to next state. Once the Q-matrix converges, we move from the training to the perception and control stage. From the converged Q-matrix, we find that the best path is to action number 3 -> 2 -> 7. Thus, we first let the robot to perform action number 3 (move green object to AR tag 1). Then, we let the robot perform action number 2 (move pink object to AR tag 3). Lastly, we let the robot perform action number 7 (move blue object to AR tag 2).

## Q-Learning Algorithm Description
1. **Selecting and executing actions for the robot (or phantom robot) to take**
- **Code Location**: Selection: (`q_learning.py`) Implemented with `a = int(self.action_matrix[self.s][next_state])` in function `q_learning()`. Execution: Implemented with `self.action_pub` in function `__init__` and `self.action_pub.publish(action_msg)` in function `q_learning()`.
- **Code Description**: Given the current state `s`, we first find valid indices of which values not equal to `-1` from `self.action_matrix[s]` using `s` as row and then we randomly pick one of them as our action and identify the next state based on its column. We then perform the action by publishing it to `/q_learning/robot_action` topic.

2. **Updating the Q-matrix**
- **Code Location**: (`q_learning.py`) Implemented with `self.q_matrix_pub` in function `__init__` and `self.q_matrix_pub.publish(q_matrix_msg)` in function `q_learning()`.
- **Code Description**: Based on robot's current action and state, next state, and reward received from the environment, we follow the Q-learing algorithm to update the Q-matrix. We choose `alpha = 1` and `gamma = 0.8` and update the Q-matrix with the formula: `q_matrix[current state][action] += alpha * (reward + gamma * (max(q_matrix[next state]) -q_matrix[current state][action]))`.


3. **Determining when to stop iterating through the Q-learning algorithm**
- **Code Location**: (`q_learning.py`) Implemented with `while no_change_count < 300 or t > 1000`  in function `q_learning()` where `no_change_count` is cumulated under statement `if  abs(self.q_matrix[self.s][a] - curr_cell) < tolerance`.
- **Code Description**: We set up `0.00001` as tolerance threshold. If the value change of the q_matrix is within tolerance, we will consider it as unchanged. Once the count of unchanged values exceeds `300` in the while loop, we consider the q_matrix converged. If the total iteration hits `1000`, then we consider we are not able to generate a q matrix and the q_learning training failed.

4. **Executing the path most likely to lead to receiving a reward after the Q-matrix has converged on the simulated Turtlebot3 robot**
- **Code Location**: Implemented with function `get_action()` in `actions.py`.
- **Code Description**: From state 0, we choose the largest value in that row as our first optimized action and add it to the `opt_action` list. Then based on this first optimized action, we update the state value (i.e., state value is a list of three values, representing the colors’ positions). We match this state value back to the state number and again, find the largest value in that row as our second optimized action, same for the third optimized action.

## Robot Perception Description
1. **Identifying the locations and identities of each of the colored objects**
- **Code Location**: Implemented with `self.detect == 0` condition in `image_callback()` function for identifying identities of the colors and `processing_scan()` function for identifying locations of the colors in `actions.py`.
- **Code Description**: When `self.detect == 0`, the robot camera starts identifying `self.curr_target` based on pre-set color bounds. We create a mask that erases all pixels that do not fall into the bounds. If the number of the color pixels we want hit a certain threthold, we would consider we find the target colored object and set a red circle as the center of the yellow pixels in the image. For the locations of the object, we take the non-zero min value of the robot's front 9-degree data scan ranges. The reason why we chose only the front 9 degrees was because we would only need the colored objects' locations when they can be seen in the camera (i.e., we spin the robot to face the color if the color isnt seen in the camera), so the object is supposed to be in front of the robot.

2. **Identifying the locations and identities of each of the AR tags**
- **Code Location**: Implemented with `self.detect == 1` condition in `image_callback()` function for identifying identities of the colors and `processing_scan()` function for identifying locations of the tags in `actions.py`.
- **Code Description**: We identify the AR tags in the same image_callback function when `self.detect` flag is equal to 1. We search for AR tags from DICT_4X4_50 in a GRAYSCALE image. For the locations of the object, we take the non-zero min value of the robot's front 9-degree data scan ranges. The reason why we chose only the front 9 degrees was because we would only need the tags' locations when they can be seen in the camera (i.e., we spin the robot to face the tag if the tag isnt seen in the camera), so the object is supposed to be in front of the robot.

## Robot Manipulation and Movement
1. **Moving to the right spot in order to pick up a colored object**
- **Code Location**: Implemented with `self.detect == 0` condition in `image_callback()` function (propotional control is after color identification) in `actions.py`.
- **Code Description**: We have a list of three optimized actions named `self.opt_actions` so each time we update the current target `self.curr_target` based on a counter as an index to access the optimized actions list. After the robot identifies the color, we use proportional control to let the robot face the center of the color object (i.e., keep the center of the colored pixels in the middle of the screen) and keep moving forward until it is close enough to the color (i.e., 0.18 meters + buffer).

2. **Picking up the colored object**
- **Code Location**: Implemented with `self.detect == 0` condition in `move_arm()` function in `actions.py`.
- **Code Description**: The robot arm is originally set to `arm_joint_goal = [0.0, math.radians(30.0), 0.0, 0.0]` where the second joint raises the entire arm up. The gripper is also set to the widest at the beginning. When the robot stopped in front of the colored object, it would first lower the second joint. Then, it would squeeze its gripper so that it could hold the colored object. Lastly, the robot would raise its arm up to the original location so that the color is not blocking the camera of the robot.

3. **Moving to the desired destination (AR tag) with the colored object**
- **Code Location**: Implemented with `self.detect == 1` condition in `image_callback()` function (propotional control is after AR tag identification) in `actions.py`.
- **Code Description**: We have a list of three optimized actions named `self.opt_actions` so each time we update the current tag `self.curr_tag` based on a counter as an index to access the optimized actions list. After the robot identifies the tag, we use proportional control to let the robot face the center of the tag (i.e., keep the center of the tag in the middle of the screen) and keep moving forward until it is close enough to the tag (i.e., 0.4 meters + buffer).

4. **Putting the colored object back down at the desired destination**
- **Code Location**: Implemented with `self.detect == 1` condition in `move_arm()` function in `actions.py`.
- **Code Description**: Once the robot is stopped around 0.4 meters away from the AR tag, the robot would again lower its second joint and place the colored object to the ground. Then, it would release its gripper so that the color can stand still on the ground. Lastly, the robot would raise its arm to the original position and step back which prevents itself from hitting the object.

## Challenges
Describe the challenges you faced and how you overcame them.

- Output csv with training.launch: We had difficulty outputting the csv file while launching the train.launch file. We eventually figured out how to solve this problem by setting the current working directory to cwd="node" in our training.launch, then it worked.
- Lagging update of the camera image: The connection of the `intro-robot` router is sometimes unstable, which caused our camera to be unable to update as quickly as possible. Whenever this happens, we need to move our color and AR tags into CSIL5 to test our implementation.
- Noise pixels in robot perception: The robot is sometimes distracted by random pixels in its environment while it is detecting the color. To solve this, we need a set threshold for the number of detected pixels to determine whether it is a paton or just some noise in the environment. 
- Location to run our implementation: When running our robot, we need to ensure that there are no other patons and AR tags near the robot. We also need to ensure that there are no similar-colored objects near the robot as well. For example, the bricks that we used to build the maze are often a distraction for the robot to recognize the color of the paton because of the bricks’ similar colors.

## Future work
If you had more time, how would you improve your implementation?

- More adjustment in robot perception: If we have more time, we would attempt to adjust the color threshold to detect each color even more precisely. For example, if we didn’t turn on the lights in CSIL 5, our robot’s color detection might be influenced. 
- More adjustment in robot’s behavior of picking targets up: We would also spend more time thinking about how to let the robot pick up the paton more precisely and smoothly. For example, in future work, we can let our robot drop the paton exactly on the ground with no tilt and in a quick amount of time.
- More adjustment in robot travel: We will experiment more with the k values that we currently implement in our proportional control for the robot's sensory motor, so that our robot can travel to the targets more smoothly and more quickly.

## Takeaways
What are your key takeaways from this project that would help you/others in future robot programming assignments working in pairs? For each takeaway, provide a few sentences of elaboration.

- Image Processing: In this project, we learned how to expand upon what we have learned in Lab B Line Follower to detect three different paton colors and three different AR tags at the same time. With this skill, we can now rely more on the robot’s camera to detect even more objects in the future, such as using it in the final project.  
- Robot Arm/Gripper control: We learned how to control the angles from the four joints of the robot arm and the width of the robot gripper in this project. This knowledge would be potentially helpful for our final project if we plan to utilize the robot arm to pick up objects or move the arm to signal gestures.
- Robot Environment Setup: A very important skill that we learned in this project is setting up the robot environment. In previous warmup projects and particle filter projects, the robot performance did not rely on the robot’s image perception. In this project, the robot highly relies on image processing to find the color and AR tag, thus we need to ensure that the robot environment does not contain similar colors to the patons and make sure that the AR tags are clearly visible (e.g. the paper of the tag is not bent or folded). These considerations will be helpful for us when setting up our robot environment for the final project.

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
