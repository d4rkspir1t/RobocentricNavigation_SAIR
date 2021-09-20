#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


def callback(data):
    posX = data.pose.position.x
    posY = data.pose.position.y
    posZ = data.pose.position.z

    oriX = data.pose.orientation.x
    oriY = data.pose.orientation.y
    oriZ = data.pose.orientation.z
    

    rospy.loginfo('(' + str(posX) + ',' + str(posY) + ',' + str(posZ) + ')   (' + str(oriX) + ',' + str(oriY) + ',' + str(oriZ) + ')')


def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', PoseStamped, callback)

    rospy.spin()


if __name__ == '__main__':
    listener()

#  ===========================
#  ~~ CMDs TO MAKE IT WORK  ~~
#  ===========================
#  roscore
#  roslaunch rosbridge_server  rosbridge_websocket.launch
#  python listener.py
