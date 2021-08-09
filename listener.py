#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

# Function to process data returned.
def callback(data):
    posX = data.pose.position.x
    posY = data.pose.position.y
    posZ = data.pose.position.z

    tup = (posX,posY,posZ)

    oriY = data.pose.orientation.y

    rospy.loginfo('Position: ' + str(posX) + ' ' + str(posY) + ' ' + str(posZ))

# Subscribes to receive data from topic 'chatter'.
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', PoseStamped, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
