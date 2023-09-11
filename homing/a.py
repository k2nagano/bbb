#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState


def callback(data):
    rospy.loginfo(data.position[0])


def listener():
    rospy.init_node('listener')
    rospy.Subscriber("/maxon/joint_states", JointState, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
