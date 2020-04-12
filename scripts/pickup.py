#! /usr/bin/env python

import rospy
import os
import sys
import geometry_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import Grasp, CollisionObject

rospy.init_node('pickup', anonymous=True)


robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
arm_torso = moveit_commander.MoveGroupCommander("arm_torso")
gripper = moveit_commander.MoveGroupCommander("gripper")

g = Grasp()
g.id = "test"

grasp_pose = PoseStamped()
grasp_pose.header.frame_id = "base_footprint"
grasp_pose.pose.position.x = 0.148554
grasp_pose.pose.position.y = -0.116075
grasp_pose.pose.position.z = 0.70493
grasp_pose.pose.orientation.x = -0.709103
grasp_pose.pose.orientation.y = 0.0137777
grasp_pose.pose.orientation.z = 0.0164031
grasp_pose.pose.orientation.w = 0.704779

right_arm.set_pose_target(grasp_pose)
right_arm.go()

rospy.sleep(2)

# set the grasp pose
g.grasp_pose = grasp_pose

# define the pre-grasp approach
g.pre_grasp_approach.direction.header.frame_id = "base_footprint"
g.pre_grasp_approach.direction.vector.x = 1.0
g.pre_grasp_approach.direction.vector.y = 0.0
g.pre_grasp_approach.direction.vector.z = 0.0
g.pre_grasp_approach.min_distance = 0.001
g.pre_grasp_approach.desired_distance = 0.1

g.pre_grasp_posture.header.frame_id = "right_gripper_base_link"
g.pre_grasp_posture.joint_names = ["gripper_left_finger_link", "gripper_right_finger_link", "gripper_link"]

pos = JointTrajectoryPoint()
pos.positions.append(0.0)

g.pre_grasp_posture.points.append(pos)

# set the grasp posture
g.grasp_posture.header.frame_id = "right_gripper_base_link"
g.grasp_posture.joint_names = ["gripper_left_finger_link", "gripper_right_finger_link", "gripper_link"]

pos = JointTrajectoryPoint()
pos.positions.append(0.2)
pos.effort.append(0.0)

g.grasp_posture.points.append(pos)

# set the post-grasp retreat
g.post_grasp_retreat.direction.header.frame_id = "base_footprint"
g.post_grasp_retreat.direction.vector.x = 0.0
g.post_grasp_retreat.direction.vector.y = 0.0
g.post_grasp_retreat.direction.vector.z = 1.0
g.post_grasp_retreat.desired_distance = 0.25
g.post_grasp_retreat.min_distance = 0.01

g.allowed_touch_objects = ["table"]

g.max_contact_force = 0

# append the grasp to the list of grasps
grasps.append(g)

rospy.sleep(2)

# pick the object
robot.right_arm.pick("part", grasps)


rospy.spin()
