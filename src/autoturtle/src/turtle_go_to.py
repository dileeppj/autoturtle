#! /usr/bin/env python3

import tf
import sys
import rospy
import roslib
roslib.load_manifest('autoturtle')
from autoturtle.srv import *
from geometry_msgs.msg import Twist, Point, Quaternion
from turtlesim.msg import Pose
from math import pow, sqrt, atan2, pi
from tf.transformations import euler_from_quaternion

class TurtleBot(object):
    """
    Class to control TurtleBot
    """
    def __init__(self):
        """
        Init the control node for turtle1 and move it to the goal
        """
        rospy.init_node("turtlebot_controller", anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.cmd_vel  = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        # self.pos_sub  = rospy.Subscriber("pose", Pose, self.update_pose)
        # self.cur_pose = Pose()
        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'
        self.connectToBot()
        (self.position, self.rotation) = self.getOdom()
        print(f"Pos: {self.position}")
        self.rate     = rospy.Rate(10)

    def connectToBot(self):
        """
        docstring
        """
        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")
        # (position, rotation) = self.getOdom()


    def getOdom(self):
        """
        Pose listener for robot object
        """
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

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

    def move2goal(self, goal_x, goal_y, tolerance):
        """
        Moves the turtle to the goal.
        """
        # self.connectToBot()
        position = Point()
        move_cmd = Twist()
        (position, rotation) = self.getOdom()
        last_rotation = 0
        linear_speed = 1
        angular_speed = 1
        goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
        distance = goal_distance

        while distance > tolerance:
            (position, rotation) = self.getOdom()
            x_start = position.x
            y_start = position.y
            # print(f"[x: {x_start}, y: {y_start}] -> {distance} => [x:{(goal_x)}, y:{(goal_y)}]")
            path_angle = atan2(goal_y - y_start, goal_x- x_start)

            if path_angle < -pi/4 or path_angle > pi/4:
                if goal_y < 0 and y_start < goal_y:
                    path_angle = -2*pi + path_angle
                elif goal_y >= 0 and y_start > goal_y:
                    path_angle = 2*pi + path_angle
            if last_rotation > pi-0.1 and rotation <= 0:
                rotation = 2*pi + rotation
            elif last_rotation < -pi+0.1 and rotation > 0:
                rotation = -2*pi + rotation
            move_cmd.angular.z = angular_speed * path_angle-rotation

            distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
            move_cmd.linear.x = min(linear_speed * distance, 0.1)

            if move_cmd.angular.z > 0:
                move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
            else:
                move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

            last_rotation = rotation
            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()
        (position, rotation) = self.getOdom()
    
    def shutdown(self):
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

def path_client(x, y):
    """
    docstring
    """
    rospy.wait_for_service('a_star')
    try:
        a_star = rospy.ServiceProxy('a_star', Path)
        res = a_star(x,y)
        return res.path
    except rospy.ServiceException as e:
        print("[ERROR] Service call failed : {e}")


    
if __name__ == "__main__":
    # x1, y1 = input("Start => x | y \n").split()
    x2, y2 = input("Goal  => x | y \n").split()
    # xx = [226,190]
    # yy = [200,170]
    # start_pos = [int(x1),int(y1)]
    try:
        turtle1 = TurtleBot()
        # start_pos = [int(turtle1.position.x/0.05), int(turtle1.position.y/0.05)
        mapGoal_pos  = [int(x2),int(y2)]
        mapStart_pos = [abs(int((turtle1.position.x -10)/0.05)),abs(int((turtle1.position.y -10)/0.05))]
        print(f"Start: {mapStart_pos} -> Goal: {mapGoal_pos}")
        res = path_client(mapStart_pos,mapGoal_pos)
        if res != '-1':
            y = res.strip('[]')
            z = y.replace(',',"")
            path = list(zip(z.split()[0::2],z.split()[1::2]))
            print("Path :",end="")
            for pos in path:
                print(f' -> [x:{int(pos[0])},y:{int(pos[1])}]', end="")
            i=1
            print('-----------------------------------------------')
            for pos in path:
                print(f'Step #{i} -> {pos}')
                turtle1.move2goal(((int(pos[0])*0.05)-10),((int(pos[1])*0.05)-10),0.2)
                i+=1
                rospy.sleep(1)
        else:
            print("No Path found")
        print("")
        rospy.loginfo("Stopping the robot...")

    except rospy.ROSInternalException:
        pass