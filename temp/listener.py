#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Pose, PoseStamped

class Listener:
    def callback(self, data):
        rospy.loginfo(data)

if __name__ == '__main__':
    # listener()
    # pose = rospy.wait_for_message('/pose_goals', Pose)
    # print(pose)
    # print('Listener done.')
    listener = Listener()
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("pose_goals", PoseStamped, listener.callback)
    rospy.spin()