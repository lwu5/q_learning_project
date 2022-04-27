# q_learning_project

Name: Timmy Lin, Liuhao Wu

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
