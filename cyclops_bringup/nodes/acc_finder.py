#!/usr/bin/env python3
import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

state = 'stopped'
start = rospy.Time(0)

LIN_MAX = 0.2
ANG_MAX = 0.5


def odomCallback(msg: Odometry):
    global state

    twist = msg.twist.twist
    t = (rospy.Time.now() - start).to_sec()

    if state == 'wait_for_stop':
        if twist.linear.x == 0.0 and twist.angular.z == 0.0:
            state = 'stopped'
            rospy.loginfo(F'state transition --> {state}')
        return

    if state == 'backward' and twist.linear.x < -0.9 * LIN_MAX:
        rospy.loginfo('backward from 0 to %f m/s in %f sec', twist.linear.x, t)
    elif state == 'forward' and twist.linear.x > 0.9 * LIN_MAX:
        rospy.loginfo('forward from 0 to %f m/s in %f sec', twist.linear.x, t)
    elif state == 'turning_clockwise' and twist.angular.z < -0.9 * ANG_MAX:
        rospy.loginfo(
            'turning_clockwise from 0 to %f rad/s in %f sec', twist.angular.z, t)
    elif state == 'turning_counter_clockwise' and twist.angular.z > 0.9 * ANG_MAX:
        rospy.loginfo(
            'turning_counter_clockwise from 0 to %f rad/s in %f sec', twist.angular.z, t)
    else:
        return

    state = 'wait_for_stop'
    rospy.loginfo('state transition --> %s', state)


def cmdvelCallback(msg: Twist):
    global state, start

    if state != 'stopped':
        return
    if msg.linear.x <= -LIN_MAX:
        start = rospy.Time.now()
        state = 'backward'
    elif msg.linear.x >= LIN_MAX:
        start = rospy.Time.now()
        state = 'forward'
    elif msg.angular.z <= -ANG_MAX:
        start = rospy.Time.now()
        state = 'turning_clockwise'
    elif msg.angular.z >= ANG_MAX:
        start = rospy.Time.now()
        state = 'turning_counter_clockwise'
    else:
        return
    rospy.loginfo(F"state transition --> {state}")


if __name__ == "__main__":
    rospy.init_node("acc_finder", anonymous=True)
    rospy.loginfo("Starting acc_finder_node.")
    rospy.Subscriber("odom", Odometry, odomCallback)
    rospy.Subscriber("cmd_vel", Twist, cmdvelCallback)
    rospy.loginfo(
        "acc_finder node ready and listening. now use teleop to move your robot to the limit!")

    rospy.spin()
