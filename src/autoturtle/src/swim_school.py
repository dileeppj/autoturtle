#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class ControlTurtleSim():
    """
    Class to control turtlesim
    """
    def __init__(self, robot):
      """
      Init the control node for turtle1 and move it with linear x velocity of 0.3
      """
      self.robot = robot
      # Create 'ControlTurtleSim' node 
      rospy.init_node('ControlTurtleSim',anonymous=False)
      # Publisher which will publish to the topic '/turtle1/cmd_vel'.
      self.vel_pub = rospy.Publisher(f'/{self.robot}/cmd_vel',Twist,queue_size=10) 
      # A subscriber to the topic '/turtle1/pose'. 
      # self.update_pose is called when a message of type Pose is received.
      self.pos_sub = rospy.Subscriber(f'/{self.robot}/pose', Pose, self.update_pose)
      self.steer = 0.5
      self.cur_pos   = Pose()
      self.start_pos = Pose()
      self.move_cmd  = Twist()         # Create obj of class Twist[geometry msg for linear & ang vel]
      self.rate = rospy.Rate(10)      # Turtlesim will receive our message in 10Hz[10 msg/sec]
      
      rospy.loginfo('Press Ctrl+C to stop the node.')
      rospy.on_shutdown(self.shutdown) # execute the function shutdown on shutdown

    def update_pose(self, data):
      """
      Pose listener for robot object
      """
      self.cur_pos = data
      self.cur_pos.x = round(self.cur_pos.x,4)
      self.cur_pos.y = round(self.cur_pos.y,4)
      # print(f'Change direction of {self.robot} at [x,y]: [{self.cur_pos.x},{self.cur_pos.y}]')

    
    def start_m1(self):
      """
      Function to start manuver #1: Basic movement in shape of 8
      """
      print(f'Change direction of {turtle1.robot} at [x,y]: [{self.start_pos.x},{self.start_pos.y}]')
      while not rospy.is_shutdown():     # Loop till Ctrl+C
        print(f'CurrPos of {self.robot} is at [x,y]: [{self.cur_pos.x},{self.cur_pos.y}]')
        print(f'IntiPos of {self.robot} is at [x,y]: [{self.start_pos.x},{self.start_pos.y}]')
        print(f'Delta [x,y]: {self.start_pos.x - self.cur_pos.x},{self.start_pos.y - self.cur_pos.y}')
        self.move_cmd.linear.x = 0.6      # Linear speed in unit/sec [+ve forward, -ve backward] 
        if self.inside_boundary(self.start_pos, 0.5):
            self.move_cmd.angular.z = self.steer    # Rotation in rad/sec
        else:
            self.move_cmd.angular.z = self.steer    # Rotation in rad/sec
        self.vel_pub.publish(self.move_cmd) # Publishing Twist values to Turtlesim node /cmd_vel
        self.rate.sleep()                   # Wait for 0.1 sec (10Hz) and publish again
  
    
    def inside_boundary(self, start_pos, distance):
      if (abs(self.cur_pos.x - start_pos.x) <= distance) and (abs(self.cur_pos.y - start_pos.y) <= distance):
          print("INSIDE")
          self.steer = self.steer * -1
          return True
      else:
        return False


    def shutdown(self):
      """
      Stopping the turtle before shutdown
      """
      rospy.loginfo('Stopping the turtle.')
      self.vel_pub.publish(Twist()) # Stopping turtlesim_move by publishing empty Twist msg
      rospy.sleep(1)                # Giving it some time to stop


if __name__ == "__main__":
    """
    Init the ControlTurtleSim class which returns the current position of the 
     turtlebot. With that position as reference, move the turtlebot with an ang vel
      of 0.3 rad/s about z axis. Once the turtlebot is back to the start pos (ref pos)
      [provide a circular boundary], rotate the turtlebot in opposit direction -0.3rad/s
    """
    try:
      turtle1 = ControlTurtleSim('turtle1')
      rospy.sleep(1)
      turtle1.start_pos = turtle1.cur_pos
      print(f'Start position[x,y] of {turtle1.robot}: {turtle1.start_pos.x},{turtle1.start_pos.y}')
      turtle1.start_m1()
    except:
      rospy.loginfo('End of swim for this turtle.')