#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
import sys

def talker():
    rospy.init_node('publish_place', anonymous=True)
    pub = rospy.Publisher('place_goals', PoseStamped, queue_size=10)

    pose = PoseStamped()
    pose.header.frame_id = "base_footprint"
    pose.pose.position.x = 0.512726171711
    pose.pose.position.y = 0.160456914268
    pose.pose.position.z = 0.253627714861
    pose.pose.orientation.x = 0.544020678534
    pose.pose.orientation.y = 0.507215268602   
    pose.pose.orientation.z = 0.500950386556
    pose.pose.orientation.w = 0.442518793765
    
    # wait until the subscriber is ready to accept the message
    rate = rospy.Rate(10) # 10hz
    while pub.get_num_connections() < 1:
        rate.sleep()
    rospy.loginfo('Publishing pose...')
    pub.publish(pose)

if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass