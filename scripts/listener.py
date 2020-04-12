#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Pose


def callback(data):
    rospy.loginfo(data)
    
def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("pose_goals", Pose, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()