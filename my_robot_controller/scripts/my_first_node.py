#!/usr/bin/env python2
import rospy

if __name__ == '__main__':
    rospy.init_node("test_node")
    rospy.loginfo("test node has been started.")

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        rospy.loginfo("Hello")
        rate.sleep()
   