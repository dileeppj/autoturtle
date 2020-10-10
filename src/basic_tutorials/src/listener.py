#! /usr/bin/env python3
import rospy
from std_msgs.msg import String

def strPrinter(data):
    """
    Prints string to screen
    """
    rospy.loginfo(rospy.get_caller_id()+" Data: %s"%data.data)

def listener():
    """
    Linstener node subscribing to 'chatter' topic
    """
    rospy.init_node('listener', anonymous=False)
    rospy.Subscriber('chatter', String, callback=strPrinter)
    # Spin keeps python from exiting till node is killed
    rospy.spin()

if __name__ == "__main__":
    listener()