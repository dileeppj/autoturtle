#! /usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():
    """
    ROS node "talker" with "chatter" publisher to publish 'Hello World!' 
    """
    pub = rospy.Publisher('chatter', String, queue_size=1)
    rospy.init_node('talker', anonymous=False)
    rate = rospy.Rate(10) # 10Hz
    while not rospy.is_shutdown():
        hello_str = "Hello World ! - %s"%rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep() 

if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass