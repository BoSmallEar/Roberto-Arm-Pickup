#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

class Listener:
    def callback(self, data):
        rospy.loginfo('Got data: ' + str(data))

# def listener():
#     rospy.init_node('listener', anonymous=True)
#     rospy.Subscriber("chatter", String, callback)
#     rospy.spin()

if __name__ == '__main__':
    print('Beginning listener...')
    listener = Listener()
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('pick_goals', PoseStamped, listener.callback)
    rospy.spin()
