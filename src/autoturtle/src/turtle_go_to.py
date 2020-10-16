#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, sqrt, atan2

class TurtleBot(object):
    """
    Class to control TurtleBot
    """
    def __init__(self):
        """
        Init the control node for turtle1 and move it to the goal
        """
        rospy.init_node("turtlebot_controller")
        self.vel_pub  = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
        self.pos_sub  = rospy.Subscriber("/turtle1/pose", Pose, self.update_pose)
        self.cur_pose = Pose()
        self.rate     = rospy.Rate(10)

    def update_pose(self, data):
        """
        Pose listener for robot object
        """
        self.cur_pose   = data
        self.cur_pose.x = round(self.cur_pose.x, 4)
        self.cur_pose.y = round(self.cur_pose.y, 4)

    def euclidean_dist(self, goal_pose):
        """
        Euclidean distance between current pose and the goal.
        """
        return sqrt(pow((goal_pose.x - self.cur_pose.x),2) + pow((goal_pose.y - self.cur_pose.y),2))

    def linear_vel(self, goal_pose, constant=1.5):
        """
        docstring
        """
        return constant*self.euclidean_dist(goal_pose)

    def steer_ang(self, goal_pose):
        """
        docstring
        """
        return atan2(goal_pose.y - self.cur_pose.y, goal_pose.x - self.cur_pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """
        docstring
        """
        return constant*(self.steer_ang(goal_pose) - self.cur_pose.theta)

    def move2goal(self, xPos, yPos, tol):
        """
        Moves the turtle to the goal.
        """
        goal_pose   = Pose()
        goal_pose.x = xPos
        goal_pose.y = yPos
        vel_msg     = Twist()

        while self.euclidean_dist(goal_pose) >= tol:
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)
            # Publishing our vel_msg
            self.vel_pub.publish(vel_msg)
            # Publish at the desired rate.
            self.rate.sleep()
        # Stopping our robot after the movement is over.
        vel_msg.linear.x  = 0
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)
        # Spin the node
        rospy.spin()

    
if __name__ == "__main__":
    try:
        turtle1 = TurtleBot()
        turtle1.move2goal(2,2,0.2)
    except rospy.ROSInternalException:
        pass