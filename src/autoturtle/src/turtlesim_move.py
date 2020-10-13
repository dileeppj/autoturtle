#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class ControlTurtleSim():
    """
    Class to control turtlesim
    """
    def __init__(self):
        """
        Init the control node for turtle1 and move it with linear x velocity of 0.3
        """
        rospy.init_node('ControlTurtleSim',anonymous=False) # ControlTurtleSim class can be init 
        # only once as it is not anonymous, ControlTurtlesim is the node name sent to the master
        rospy.loginfo('Press Ctrl+C to stop the node.')
        rospy.on_shutdown(self.shutdown) # execute the function shutdown on shutdown
        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel',Twist,queue_size=10) # Creating a handle
        # to publish messages to the the topic '/turtle1/cmd_vel'
        rate = rospy.Rate(10)      # Turtlesim will receive our message in 10Hz[10 msg/sec]
        move_cmd = Twist()         # Create obj of class Twist[geometry msg for linear & ang vel]
        move_cmd.linear.x = 0.3    # Linear speed in unit/sec [+ve forward, -ve backward] 
        move_cmd.angular.z = 0     # Rotation in rad/sec
        while not rospy.is_shutdown():     # Loop till Ctrl+C
            self.cmd_vel.publish(move_cmd) # Publishing Twist values to Turtlesim node /cmd_vel
            rate.sleep()                   # Wait for 0.1 sec (10Hz) and publish again

    def shutdown(self):
        """
        Stopping the turtle before shutdown
        """
        rospy.loginfo('Stopping the turtle.')
        self.cmd_vel.publish(Twist()) # Stopping turtlesim_move by publishing empty Twist msg
        rospy.sleep(1)                # Giving it some time to stop


if __name__ == "__main__":
    try:
        ControlTurtleSim()
    except:
        rospy.loginfo('End of swim for this turtle.')