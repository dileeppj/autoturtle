#! /usr/bin/env python3

import sys
import roslib
roslib.load_manifest('autoturtle')

import rospy
from autoturtle.srv import *

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
    x = [0,0]
    y = [5,7]
    res = path_client(x,y)
    # res = res.split() 
    print(res)
    # x = [item for t in res for item in t] 
    # y = str(x)
    y = res.strip('[]')
    z = y.replace(',',"")
    z = list(zip(z.split()[0::2],z.split()[1::2]))
    print(z[1][1])
    # res = list(zip(temp[::2], temp[1::2])) 
    # print(f"[INFO] Result = {x} -> {y} = {res}")
    print(f"[INFO] Result = {x} -> {y} = {z}")

