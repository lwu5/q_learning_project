#!/usr/bin/env python3

import rospy
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math


class Robot(object):

  def __init__(self):

      # initialize this node
      rospy.init_node('turtlebot3_dance')

      # the interface to the group of joints making up the turtlebot3
      # openmanipulator arm
      self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

      # the interface to the group of joints making up the turtlebot3
      # openmanipulator gripper
      self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

      # Reset arm position
      self.move_group_arm.go([0,0,0,0], wait=True)
      
      print("ready")
      
  def run(self):

      # We can use the following function to move the arm
      # self.move_group_arm.go(arm_joint_goal, wait=True)

      # arm_joint_goal is a list of 4 radian values, 1 for each joint
      # wait=True ensures that the movement is synchronous

      # Let's move the arm based on what we have learned

      # First determine at what angle each joint should be.
      # You can use the GUI to find appropriate values based on your need.
      rospy.sleep(2)
      # Move the arm

      #position to pick up
      arm_joint_goal = [
        0.0,
        math.radians(20.0),
        0.0,#math.radians(10.0),
        0.0#math.radians(-20.0)
      ]

      self.move_group_arm.go(arm_joint_goal, wait=True)

      # The above should finish once the arm has fully moved.
      # However, to prevent any residual movement,we call the following as well.
      self.move_group_arm.stop()  

      # We can use the following function to move the gripper
      # self.move_group_gripper.go(gripper_joint_goal, wait=True)

      # gripper_joint_goal is a list of 2 values in meters, 1 for the left gripper and one for the right
      # wait=True ensures that the movement is synchronous

      # Let's move the gripper based on what we have learned

      # First determine what how far the grippers should be from the base position.
      # You can use the GUI to find appropriate values based on your need.
      gripper_joint_goal = [0.009,0.0009]

      # Move the gripper
      self.move_group_gripper.go(gripper_joint_goal, wait=True)

      # The above should finish once the arm has fully moved.
      # However, to prevent any residual movement,we call the following as well.
      self.move_group_gripper.stop()

      rospy.sleep(3)

      #position to pick up and squeeze
      gripper_joint_goal = [0.000,0.0000]

      # Move the gripper
      self.move_group_gripper.go(gripper_joint_goal, wait=True)

      # The above should finish once the arm has fully moved.
      # However, to prevent any residual movement,we call the following as well.
      self.move_group_gripper.stop()

      rospy.sleep(3)

      #pick up
      arm_joint_goal = [
        0.0,
        0.0,
        0.0,#math.radians(10.0),
        0.0#math.radians(-20.0)
      ]
      self.move_group_arm.go(arm_joint_goal, wait=True)

      # The above should finish once the arm has fully moved.
      # However, to prevent any residual movement,we call the following as well.
      self.move_group_arm.stop()  


      #drop the paton
      rospy.sleep(3)

      
      arm_joint_goal = [
        0.0,
        math.radians(20.0),
        0.0,#math.radians(10.0),
        0.0#math.radians(-20.0)
      ]
      self.move_group_arm.go(arm_joint_goal, wait=True)

      # The above should finish once the arm has fully moved.
      # However, to prevent any residual movement,we call the following as well.
      self.move_group_arm.stop() 

      rospy.sleep(3)

      #position to pick up and squeeze
      gripper_joint_goal = [0.009,0.0009]

      # Move the gripper
      self.move_group_gripper.go(gripper_joint_goal, wait=True)

      # The above should finish once the arm has fully moved.
      # However, to prevent any residual movement,we call the following as well.
      self.move_group_gripper.stop()

      rospy.sleep(3)

      arm_joint_goal = [
        0.0,
        0.0,
        0.0,#math.radians(10.0),
        0.0#math.radians(-20.0)
      ]
      self.move_group_arm.go(arm_joint_goal, wait=True)

      # The above should finish once the arm has fully moved.
      # However, to prevent any residual movement,we call the following as well.
      self.move_group_arm.stop() 


      rospy.sleep(3)




      print("done!")


if __name__ == "__main__":
    robot = Robot()
    robot.run()