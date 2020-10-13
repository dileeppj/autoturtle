#! /usr/bin/env python3

import roslib
roslib.load_manifest('basic_tutorials')

import sys
import rospy
from basic_tutorials.srv import *

def add_two_ints_client(x, y):
    """
    docstring
    """
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        res = add_two_ints(x,y)
        return res.sum
    except rospy.ServiceException as e:
        print("[ERROR] Service call failed : {e}")


if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print("[INFO] Usage: {sys.argv[0]} [x y]")
        sys.exit(1)
    
    print(f"[INFO] Requesting {x} + {y}")
    print(f"[INFO] Result = {x} + {y} = {add_two_ints_client(x,y)}")