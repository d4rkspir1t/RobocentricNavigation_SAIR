#!/usr/bin/env python
"""
Script to move Turtlesim in a circle
"""
import rospy
from geometry_msgs.msg import Twist


def move_circle():
    # Create a publisher which can "talk" to Turtlesim and tell it to move
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # Create a Twist message and add linear x and angular z values
    move_cmd = Twist()
    move_cmd.linear.x = 2.0
    move_cmd.angular.z = 20.0

    # Save current time and set publish rate at 10 Hz
    now = rospy.Time.now()
    rate = rospy.Rate(10)

    # For the next 6 seconds publish cmd_vel move commands to Turtlesim
    while rospy.Time.now() < now + rospy.Duration.from_sec(300):
        pub.publish(move_cmd)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('velocity_control', anonymous=True)
    try:
        move_circle()
    except rospy.ROSInterruptException:
        pass