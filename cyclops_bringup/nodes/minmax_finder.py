#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

min_twist = Twist()
max_twist = Twist()


def odomCallback(msg: Odometry):
    global min_twist, max_twist

    twist = msg.twist.twist

    if min_twist.linear.x > twist.linear.x:
        min_twist.linear.x = twist.linear.x
    if max_twist.linear.x < twist.linear.x:
        max_twist.linear.x = twist.linear.x

    if min_twist.linear.y > twist.linear.y:
        min_twist.linear.y = twist.linear.y
    if max_twist.linear.y < twist.linear.y:
        max_twist.linear.y = twist.linear.y

    if min_twist.angular.z > twist.angular.z:
        min_twist.angular.z = twist.angular.z
    if max_twist.angular.z < twist.angular.z:
        max_twist.angular.z = twist.angular.z

    rospy.loginfo('linear: X [%f, %f] Y [%f. %f] angular: Z [%f, %f]', min_twist.linear.x,
                  max_twist.linear.x, min_twist.linear.y, max_twist.linear.y, min_twist.angular.z, max_twist.angular.z)


if __name__ == "__main__":
    rospy.init_node("minmax_finder", anonymous=True)
    rospy.loginfo("Starting minmax_finder_node.")
    rospy.Subscriber("odom", Odometry, odomCallback)
    rospy.loginfo(
        "min_max_finde node ready and listening. now use teleop to move your robot to the limits!")

    rospy.spin()
