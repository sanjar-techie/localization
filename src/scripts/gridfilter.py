#!/usr/bin/env python2

from __future__ import print_function
import rospy
import math
import numpy as np
import sys
import signal

from std_msgs.msg import Header, Time
from nav_msgs.msg import OccupancyGrid, MapMetaData

# global variables because we're lazy
grid = OccupancyGrid()
grid_grabbed = False

GRID_THRESHOLD = 65 # over this is an obstacle :)
MAP_TOPIC_IN = '/map_in'
MAP_TOPIC_OUT = '/map_out'

# https://stackoverflow.com/questions/40438261/using-sigint-to-kill-a-function-in-python-3
class SigIntException(BaseException): pass

def stop(signal, frame):
    print("You pressed ctrl-c")
    raise SigIntException

def grid_cb(msg):
    global grid, grid_grabbed
    try:
        grid = msg
        grid_grabbed = True
    except SigIntException:
            return

def ros_node_main(rate):
    global grid, grid_grabbed, GRID_THRESHOLD, MAP_TOPIC_IN, MAP_TOPIC_OUT

    rospy.init_node('gridfilter_node', anonymous=True)
    rate = rospy.Rate(rate) # 1 Hz by default
    map_sub = rospy.Subscriber(MAP_TOPIC_IN, OccupancyGrid, grid_cb, queue_size=1)
    map_pub = rospy.Publisher(MAP_TOPIC_OUT, OccupancyGrid, queue_size=1)
    rospy.loginfo('Filtering grid')

    try:
        while not rospy.is_shutdown():
            if (grid_grabbed): # grid has been received!
                # http://docs.ros.org/en/jade/api/ros_numpy/html/occupancy__grid_8py_source.html 
                # make grid variables:
                width = grid.info.width
                height = grid.info.height

                if (width > 0 and height > 0): # otherwise, no grid to filter
                    if (width * height == len(grid.data)): # dimensions are consistent
                        map_to_pub = grid # init as original grid
                        grid_line = np.array(np.array(map_to_pub.data)> GRID_THRESHOLD, dtype=int) # convert to binary np array
                        grid_rect = grid_line.reshape((height, width)) # convert to rectangle
                        grid_padd = np.zeros((height+2, width+2))
                        grid_padd[1:np.shape(grid_padd)[0]-1, 1:np.shape(grid_padd)[1]-1] = grid_rect
                        # # 9-point check:
                        # grid_sums = grid_padd[0:np.shape(grid_padd)[0]-2, 0:np.shape(grid_padd)[1]-2] +\
                        #             grid_padd[1:np.shape(grid_padd)[0]-1, 0:np.shape(grid_padd)[1]-2] +\
                        #             grid_padd[2:np.shape(grid_padd)[0]-0, 0:np.shape(grid_padd)[1]-2] +\
                        #             grid_padd[0:np.shape(grid_padd)[0]-2, 1:np.shape(grid_padd)[1]-1] +\
                        #             grid_padd[1:np.shape(grid_padd)[0]-1, 1:np.shape(grid_padd)[1]-1] +\
                        #             grid_padd[2:np.shape(grid_padd)[0]-0, 1:np.shape(grid_padd)[1]-1] +\
                        #             grid_padd[0:np.shape(grid_padd)[0]-2, 2:np.shape(grid_padd)[1]-0] +\
                        #             grid_padd[1:np.shape(grid_padd)[0]-1, 2:np.shape(grid_padd)[1]-0] +\
                        #             grid_padd[2:np.shape(grid_padd)[0]-0, 2:np.shape(grid_padd)[1]-0]
                        # 5-point check:
                        grid_sums = grid_padd[1:np.shape(grid_padd)[0]-1, 0:np.shape(grid_padd)[1]-2] +\
                                    grid_padd[0:np.shape(grid_padd)[0]-2, 1:np.shape(grid_padd)[1]-1] +\
                                    grid_padd[1:np.shape(grid_padd)[0]-1, 1:np.shape(grid_padd)[1]-1] +\
                                    grid_padd[2:np.shape(grid_padd)[0]-0, 1:np.shape(grid_padd)[1]-1] +\
                                    grid_padd[1:np.shape(grid_padd)[0]-1, 2:np.shape(grid_padd)[1]-0]
                        grid_filt = np.multiply(np.array(grid_sums>1, dtype=int), grid_rect)
                        map_to_pub.data = grid_filt.flatten() * 101 - 1
                        # Publish:
                        map_pub.publish(map_to_pub)
                        rospy.loginfo('Filtered map published.')
                    else:
                        rospy.loginfo('Dimension error! Quitting.')
                else:
                    rospy.loginfo('Map received has null dimension! Skipping.')
                grid_grabbed = False # clear flag :)
            rate.sleep()
        rospy.spin() # spin() simply keeps python from exiting until this node is stopped
    except SigIntException:
        rospy.loginfo('Quit received.')

if __name__ == '__main__':
    try:
        rate = 1
        for argv_element in sys.argv:
            try: 
                float(argv_element)
            except ValueError:
                continue
            rate = float(argv_element)
            break
        ros_node_main(rate)
    except rospy.ROSInterruptException:
        pass
