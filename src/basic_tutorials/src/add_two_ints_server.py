#! /usr/bin/env python3

import roslib
roslib.load_manifest('basic_tutorials')

import rospy
from basic_tutorials.srv import *

def handle_add_two_ints(req):
    """
    Returns the value req.sum as a result od req.a+req.b
    AddTwoIntsRequest and AddTwoIntsResponse are formed when making the custom srv
    AddTwoInts.srv by ROS. When we say return(AddTwoIntsResponse), it will return 
    sum in the custom *.srv data structure 
    """
    print(f"[INFO] Returning {req.a} + {req.b} = {req.a+req.b} on obj.sum")
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    """
    Init the node and start the service 'add_two_ints' using custom srv file
    """
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handler=handle_add_two_ints)
    print("[INFO] Ready to add two ints.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()