#! /usr/bin/env python3
import rospy
from std_msgs.msg import String
from turtlesim.msg import Pose

def callback_listener(data):
    """
    callback listener to print the pose
    """
    rospy.loginfo(f'Turtle1 => x : {data.x}, y : {data.y}')

def listener():
    """
    Listener node to listen to pose message from turtle1
    """
    rospy.init_node('tsim_listener',anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, callback_listener)
    rospy.spin()

if __name__ == "__main__":
    listener()