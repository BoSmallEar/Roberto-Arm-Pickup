#!/usr/bin/env python

# Copyright (c) 2016 PAL Robotics SL. All Rights Reserved
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted, provided that the
# above copyright notice and this permission notice appear in all
# copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#
# Author:
#   * Sam Pfeiffer
#   * Job van Dieten
#   * Jordi Pages
#
# Modified version of Tiago Pick and Place tutorial for EECS 467 Final project
#

import rospy
import time
from tiago_pick_demo.msg import PickUpPoseAction, PickUpPoseGoal
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from actionlib import SimpleActionClient

import tf2_ros
from tf2_geometry_msgs import do_transform_pose

import numpy as np
from std_srvs.srv import Empty

import cv2
from cv_bridge import CvBridge

import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import MoveItErrorCodes


moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name


class PickupHandler(object):
    def __init__(self):
        # maximum height of the torso
        self.HEIGHT_MAX = rospy.get_param('~torso_height_limit')

        '''Start the action client services and initialize publishers.'''
        rospy.loginfo("Initalizing...")
        self.bridge = CvBridge()
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_l = tf2_ros.TransformListener(self.tfBuffer)
                
        rospy.loginfo("Waiting for /pickup_pose AS...")
        self.pick_as = SimpleActionClient('/pickup_pose', PickUpPoseAction)
        time.sleep(1.0)
        if not self.pick_as.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to /pickup_pose AS")
            exit()
        rospy.loginfo("Waiting for /place_pose AS...")
        self.place_as = SimpleActionClient('/place_pose', PickUpPoseAction)

        self.place_as.wait_for_server()

        # Initialize publishers
        rospy.loginfo("Setting publishers to torso and head controller...")
        self.torso_cmd = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1)
        self.head_cmd = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
        self.detected_pose_pub = rospy.Publisher('/detected_aruco_pose',
            PoseStamped, queue_size=1, latch=True)

        rospy.loginfo("Waiting for '/play_motion' AS...")
        self.play_m_as = SimpleActionClient('/play_motion', PlayMotionAction)
        if not self.play_m_as.wait_for_server(rospy.Duration(20)):
            rospy.logerr("Could not connect to /play_motion AS")
            exit()
        rospy.loginfo("Connected!")


        # Connect to MoveIt Commander
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander('arm_torso')
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self.eef_link = self.group.get_end_effector_link()
        self.goals = []
        
        rospy.sleep(1.0)
        rospy.loginfo("Done initializing PickupHandler.")
    

    def lift_torso(self, height=0.0):
        '''
        Move the torso to the desired height.

        If height is less than 0 or greater than HEIGHT_MAX, height is 
        set to those values.
        '''
        rospy.loginfo("Moving torso up")
        jt = JointTrajectory()
        jt.joint_names = ['torso_lift_joint']
        jtp = JointTrajectoryPoint()
        height = min(self.HEIGHT_MAX, max(0.0, height))

        jtp.positions = [height]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.torso_cmd.publish(jt)


    def lower_head(self):
        '''
        Lower robot head.
        
        There's currently no purpose to this, it just looks cool.
        '''
        rospy.loginfo("Lowering head for a e s t h e t i c")
        jt = JointTrajectory()
        jt.joint_names = ['head_1_joint', 'head_2_joint']
        jtp = JointTrajectoryPoint()
        jtp.positions = [0.0, -1.0]
        jtp.time_from_start = rospy.Duration(2.0)
        jt.points.append(jtp)
        self.head_cmd.publish(jt)


    def move_arm_to_final(self):
        '''
        Moves arm to the final pose after a successful pickup.

        The final pose of the arm is given in config/pick_motions.yaml
        as 'pick_final_pose'. Currently this flips the arm over causing
        the object to fall...
        '''
        rospy.loginfo("Moving arm to a safe pose")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'pick_final_pose'
        pmg.skip_planning = False
        self.play_m_as.send_goal_and_wait(pmg)
        rospy.loginfo("Raise object done.")
        rospy.loginfo("Done!")


    def pickup(self, goal_pose):
        '''
        Move the gripper to goal_pose and pick up the object at that position.

        Parameters:
        goal_pose:  PoseStamped object indicating the position of the object
                    to be picked up. Currently the frame of the object has to
                    be 'base_footprint'
        '''

        # Remove leading slash from frame id
        goal_pose.header.frame_id = self.strip_leading_slash(goal_pose.header.frame_id)
        rospy.loginfo("Goal received:\n" + str(goal_pose))
        self.prepare_robot()
        rospy.sleep(2.0)

        # Create and initialize pickup goal with position from goal pose
        # TODO: if the goal pose frame is not in base, need to transform
        pick_g = PickUpPoseGoal()
        pick_g.object_pose.pose.position = goal_pose.pose.position
        pick_g.object_pose.pose.position.z -= 0.1*(1.0/2.0)
        # rospy.loginfo("goal pose in base_footprint:" + str(pick_g))
        pick_g.object_pose.header.frame_id = 'base_footprint'
        pick_g.object_pose.pose.orientation.w = 1.0
        self.detected_pose_pub.publish(pick_g.object_pose)

        # Send pickup goal to pick and place server
        rospy.loginfo("Attempting to pick up:" + str(pick_g))
        self.pick_as.send_goal_and_wait(pick_g)
        rospy.loginfo("Done!")

        # Check that the pickup was successful
        # TODO: what to do if pickup wasn't successful??
        result = self.pick_as.get_result()
        if str(moveit_error_dict[result.error_code]) != "SUCCESS":
            rospy.logerr("Failed to pick, not trying further - error code was {}"
                .format(str(moveit_error_dict[result.error_code])))
            # return

        # move arm to final position
        self.lift_torso(height=0.2)
        # self.move_arm_to_final() # pick_final_pose causes object to fall

    def place(self, goal_pose):
        
        # move arm to desired pose
        # get goal and plan path
        # rospy.loginfo('Planning path to new goal...')
        # self.group.set_pose_target(goal_pose)
        # plan = self.group.plan()

        # # # visualize path in rviz
        # # rospy.loginfo('Visualizing in RViz...')
        # # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        # # display_trajectory.trajectory_start = self.robot.get_current_state()
        # # display_trajectory.trajectory.append(plan)
        # # self.display_trajectory_publisher.publish(display_trajectory)
        # # rospy.sleep(2) # allow time for visualization

        # # perform movement
        # self.group.go(wait=True)

        # print('Path planning and movement successful!')

        # # clear current goal
        # self.group.stop()
        # self.group.clear_pose_targets()

        # open gripper so object falls out
        # self.open_gripper()

        # Remove leading slash from frame id
        goal_pose.header.frame_id = self.strip_leading_slash(goal_pose.header.frame_id)
        rospy.loginfo("Goal received:\n" + str(goal_pose))

        # Create and initialize pickup goal with position from goal pose
        pick_g = PickUpPoseGoal()
        pick_g.object_pose.pose.position = goal_pose.pose.position

        # rospy.loginfo("goal pose in base_footprint:" + str(pick_g))
        pick_g.object_pose.header.frame_id = 'base_footprint'
        pick_g.object_pose.pose.orientation.w = 1.0
        
        rospy.loginfo("Placing object at goal pose")
        self.place_as.send_goal_and_wait(pick_g)




    def prepare_robot(self):
        '''Prepare the robot for picking up an object.'''
        self.unfold_arm()
        self.lower_head()
        rospy.loginfo("Robot prepared.")


    def unfold_arm(self):
        '''
        Unfolds the robot arm.

        The configuration of the unfolded arm is given in
        config/pick_motions.yaml as 'pregrasp'.
        '''
        rospy.loginfo("Unfold arm safely")
        pmg = PlayMotionGoal()
        pmg.motion_name = 'pregrasp'
        pmg.skip_planning = False
        self.play_m_as.send_goal_and_wait(pmg)
        rospy.loginfo("Done unfolding arm.")


    def open_gripper(self):
        '''
        Opens the gripper.

        The configuration is given somewhere in tiago_gazebo...
        '''
        rospy.loginfo('Opening gripper')
        pmg = PlayMotionGoal()
        pmg.motion_name = 'open'
        pmg.skip_planning = False
        self.play_m_as.send_goal_and_wait(pmg)
        rospy.loginfo('Done opening gripper')


    def strip_leading_slash(self, s):
        return s[1:] if s.startswith("/") else s


if __name__ == '__main__':
    rospy.init_node('pickup')

    # Create pickup handler and begin listening for a new goal
    picker = PickupHandler()
    rospy.Subscriber('pick_goals', PoseStamped, picker.pickup)
    rospy.Subscriber('place_goals', PoseStamped, picker.place)
    rospy.loginfo('Waiting for pick goal...')

    rospy.spin()

