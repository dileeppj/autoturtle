#! /usr/bin/env python3 
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

def mapCallback(mapData):
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
    np_data = (np_data/100).astype(int)
    print(f'Shape of data: {np_data.shape}')
    np.savetxt('./map.txt',np_data, fmt='%1.0e')

def mapListener():
    """
    docstring
    """
    rospy.init_node('getMap', anonymous=False)
    rospy.Subscriber('map', OccupancyGrid, callback=mapCallback)
    rospy.spin()

if __name__ == "__main__":
    mapListener()