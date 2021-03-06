#! /usr/bin/env python3

import math
import sys
import rospy
import roslib
import numpy as np
from PIL import Image
from turtlesim.msg import Pose
from nav_msgs.msg import OccupancyGrid
roslib.load_manifest('autoturtle')
from autoturtle.srv import *


class PathPlanner(object):
    def __init__(self):
        """
        List of lists that represents the occupancy map/grid. 
        List should only contain 0's for open nodes 
        and 1's for obstacles/walls.
        """
        # self.grid      = grid
        self.heuristic = None
        self.goal_node = None

    def getMap(self):
        """
        docstring
        """
        rospy.Subscriber('map', OccupancyGrid, callback=self.mapCallback)

    def mapCallback(self,mapData):
        """
        docstring
        """
        width = mapData.info.width
        height = mapData.info.height
        resolution = mapData.info.resolution
        data = mapData.data
        print(f'Width: {mapData.info.width}, Height: {mapData.info.height}')
        print(f'Type of Data: {type(data)}')
        print(f'Leng of Data: {len(data)}')
        np_data = np.asarray(data)
        np_data = np.reshape(np_data, (width,height))
        np_data = np.where(np_data==-1,100,np_data)
        np_data = np_data/100
        np_data = np_data.astype(int)
        self.grid = np_data.tolist()

    def calc_heuristic(self):
        """
        Function will create a list of lists the same size
        of the occupancy map, then calculate the cost from the
        goal node to every other node on the map and update the
        class member variable self.heuristic.
        """
        row = len(self.grid)
        col = len(self.grid[0])
        
        self.heuristic = [[0 for x in range(col)] for y in range(row)]
        for i in range(row):
            for j in range(col):
                row_diff = abs(i - self.goal_node[0])
                col_diff = abs(j - self.goal_node[1])
                self.heuristic[i][j] = int(abs(row_diff - col_diff)+min(row_diff,col_diff)*2)
        # print("Heuristic:")                
        # for i in range(len(self.heuristic)):
        #     print(self.heuristic[i])

    def a_star(self, start_pos, goal_pos):
        """
        A* Planner method. Finds a plan from a starting node
        to a goal node if one exits.
        :param init: Initial node in an Occupancy map. [x, y].
        Type: List of Ints.
        :param goal: Goal node in an Occupancy map. [x, y].
        Type: List of Ints.
        :return: Found path or -1 if it fails.
        """
        goal = [int(goal_pos[1]), int(goal_pos[0])]
        self.goal_node = goal
        init = [int(start_pos[1]),int(start_pos[0])]
        self.calc_heuristic()
        print(init, goal)

        delta = [[-1, 0],  # go up
                 [0 ,-1],  # go left
                 [1 , 0],  # go down
                 [0 , 1]]  # go right
        delta_name = ['^ ', '< ', 'v ', '> ']

        closed = [[0 for col in range(len(self.grid[0]))] for row in range(len(self.grid))]
        shortest_path = [[0 for _ in range(len(self.grid[0]))] for _ in range(len(self.grid))]
        closed[init[0]][init[1]] = 1

        expand = [[-1 for col in range(len(self.grid[0]))] for row in range(len(self.grid))]
        delta_tracker = [[-1 for _ in range(len(self.grid[0]))] for _ in range(len(self.grid))]

        cost = 1
        x = init[0]
        y = init[1]
        g = 0
        f = g + self.heuristic[x][y]
        open = [[f, g, x, y]]

        found = False  # flag that is set when search is complete
        resign = False  # flag set if we can't find expand
        count = 0
        while not found and not resign:
            if len(open) == 0:
                resign = True
                return -1
            else:
                open.sort()
                open.reverse()
                next = open.pop()
                x = next[2]
                y = next[3]
                g = next[1]
                expand[x][y] = count
                count += 1

                if x == goal[0] and y == goal[1]:
                    found = True
                else:
                    for i in range(len(delta)):
                        x2 = x + delta[i][0]
                        y2 = y + delta[i][1]
                        if len(self.grid) > x2 >= 0 <= y2 < len(self.grid[0]):
                            if closed[x2][y2] == 0 and self.grid[x2][y2] == 0:
                                g2 = g + cost
                                f = g2 + self.heuristic[x2][y2]
                                open.append([f, g2, x2, y2])
                                closed[x2][y2] = 1
                                delta_tracker[x2][y2] = i

        current_x = goal[0]
        current_y = goal[1]
        start_x = init[0]
        start_y = init[1]
        shortest_path[current_x][current_y] = 255
        full_path = []
        while current_x != init[0] or current_y != init[1]:
            previous_x = current_x - delta[delta_tracker[current_x][current_y]][0]
            previous_y = current_y - delta[delta_tracker[current_x][current_y]][1]
            shortest_path[previous_x][previous_y] = 150
            full_path.append((current_x, current_y))
            current_x = previous_x
            current_y = previous_y
        shortest_path[start_x][start_y] = 200
        full_path.reverse()
        # print( "Found the goal in {} iterations.".format(count))
        # print( "full_path: ", full_path[:-1])
        # for i in range(len(shortest_path)):
        #     print( shortest_path[i])
        # data = np.array(shortest_path)
        # rescaled = (255.0 / data.max() * (data - data.min())).astype(np.uint8)
        # im = Image.fromarray(rescaled)
        # im.save('path.png')
        path = np.array(shortest_path)
        data = np.array(self.grid)
        # data[data<0]=1
        r_data = (255.0 / data.max() * (data - data.min())).astype(np.uint8)
        r_path = (255.0 / path.max() * (path - path.min())).astype(np.uint8)
        im_data = Image.fromarray(r_data)
        im_path = Image.fromarray(r_path)
        im_data.paste(im_path, (0, 0), im_path)
        im_data.save('data.png')
        return full_path[:-1]

def handler_astar(req):
    print(req.start, req.stop)
    # print(type(req.start))
    # print(type(req))
    res = planner.a_star(req.start, req.stop)
    if res != -1:
        path = [item for t in res for item in t]
        # print(type(path))
        # print(path)
        return str(path)
    else:
        print("No Path found")
        return str(-1)
    # return str([list(ele) for ele in res]) 
    # print(type(res))x
    # return res

if __name__ == '__main__':
    planner = PathPlanner()
    planner.getMap()
    rospy.init_node('path_planner')
    # planner.a_star(test_start, test_goal)
    s = rospy.Service('a_star', Path, handler=handler_astar)
    print("[INFO] Ready to find Path (A-Star).")
    rospy.spin()
