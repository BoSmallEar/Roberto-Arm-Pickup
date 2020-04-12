#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Pose
import sys

def talker():
    pub = rospy.Publisher('pose_goals', Pose, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     hello_str = 'hello_world'.format(rospy.get_time())
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)
    #     rate.sleep()
    pose = Pose()
    pose.position.x = float(sys.argv[1])
    pose.position.y = float(sys.argv[2])
    pose.position.z = float(sys.argv[3])
    pose.orientation.x = float(sys.argv[4])
    pose.orientation.y = float(sys.argv[5])
    pose.orientation.z = float(sys.argv[6])
    pose.orientation.w = 1.0
    pub.publish(pose)


if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass