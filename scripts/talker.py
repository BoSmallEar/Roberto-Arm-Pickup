#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Pose, PoseStamped
import sys

def talker():
    pub = rospy.Publisher('pick_goals', PoseStamped, queue_size=10)
    rospy.init_node('talker', anonymous=True)

    pose = PoseStamped()
    pose.header.frame_id = "base_footprint"
    pose.pose.position.x = 0.492726171711
    pose.pose.position.y = -0.140456914268
    pose.pose.position.z = 0.133627714861
    pose.pose.orientation.x = 0.544020678534
    pose.pose.orientation.y = 0.507215268602   
    pose.pose.orientation.z = 0.500950386556
    pose.pose.orientation.w = 0.442518793765

    # pose.position.x = float(sys.argv[1])
    # pose.position.y = float(sys.argv[2])
    # pose.position.z = float(sys.argv[3])
    # pose.orientation.x = float(sys.argv[4])
    # pose.orientation.y = float(sys.argv[5])
    # pose.orientation.z = float(sys.argv[6])
    # pose.orientation.w = 1.0
    rospy.loginfo('Publishing pose...')
    pub.publish(pose)


if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass