#! /usr/bin/env python

import rospy
import os
import sys
import geometry_msgs.msg
from geometry_msgs.msg import Pose
import moveit_commander
import moveit_msgs.msg


class Planner:
	def __init__(self, name):
		# initialize ros node
		rospy.init_node('ik_planner', log_level=rospy.INFO) 

		# get move group info
		try:
			self.robot = moveit_commander.RobotCommander()
			self.scene = moveit_commander.PlanningSceneInterface()
			self.group = moveit_commander.MoveGroupCommander(name)
			self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
													moveit_msgs.msg.DisplayTrajectory, queue_size=10)

			self.eef_link = self.group.get_end_effector_link()

		except RuntimeError as e:
			rospy.loginfo(e)
			sys.exit(1)

		self.goals = []
		rospy.loginfo('Planner initialized successfully.')


	def get_current_goals():
		return self.goals

	
	def goal_received(self, pose):
		self.goals.append(pose)
		rospy.loginfo('New goal received at time {}'.format(rospy.get_time()))

	
	def has_current_goal(self):
		return len(self.goals) != 0

	
	def plan_and_go(self):
		if self.has_current_goal():
			# get goal and plan path
			rospy.loginfo('Planning path to new goal...')
			goal = self.goals.pop(0)
			self.group.set_pose_target(goal)
			plan = self.group.plan()

			# visualize path in rviz
			rospy.loginfo('Visualizing in RViz...')
			display_trajectory = moveit_msgs.msg.DisplayTrajectory()
			display_trajectory.trajectory_start = self.robot.get_current_state()
			display_trajectory.trajectory.append(plan)
			self.display_trajectory_publisher.publish(display_trajectory)
			rospy.sleep(2) # allow time for visualization

			# perform movement
			self.group.go(wait=True)

			print('Path planning and movement successful!')
			print('Current pose: ', self.group.get_current_pose().pose)

			# clear current goal
			self.group.stop()
			self.group.clear_pose_targets()


if __name__=='__main__':
	planner = Planner('arm_torso')
	rospy.Subscriber("pose_goals", Pose, planner.goal_received)
	while not rospy.is_shutdown():
		if planner.has_current_goal():
			planner.plan_and_go()
		try:
			rospy.sleep(1)
		except rospy.ROSInterruptException:
			continue
	
