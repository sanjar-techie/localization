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
MAP_IN_TOPIC = '/map/truepath'
MAP_OUT_TOPIC = '/map/out'
NUM_CELLS_TO_PAD = 1

def grid_cb(msg):
    global grid, grid_grabbed, MAP_TOPIC, pc_ready
    try:
        grid = msg
        grid_grabbed = True
        pc_ready = False
        #rospy.loginfo('Grid received on ' + MAP_TOPIC + '! Stored.')
    except SigIntException:
            return

def ros_node_main(rate):
    global grid, grid_grabbed, GRID_THRESHOLD, MAP_IN_TOPIC, MAP_OUT_TOPIC

    rospy.init_node('grid2pc_node', anonymous=True, disable_signals=True)
    rate = rospy.Rate(rate) # 1 Hz by default
    sub = rospy.Subscriber(MAP_IN_TOPIC, OccupancyGrid, grid_cb, queue_size=1)
    pub = rospy.Publisher(MAP_OUT_TOPIC, OccupancyGrid, queue_size=1)
    rospy.loginfo('Inflating grid.')

    while not rospy.is_shutdown():
        rate.sleep()

        if not grid_grabbed: # grid has not been received
            continue

        # make grid variables:
        grid_width = grid.info.width
        grid_height = grid.info.height

        if (grid_width < 1 or grid_height < 1): # bad grid
            rospy.loginfo('Map received has bad dimension/s! Skipping.')
            grid_grabbed = False
            continue

        map_data = np.asarray(grid.data, dtype=np.int8)
        
        if not (grid_width * grid_height == len(map_data)): # dim is bad!
            rospy.loginfo('Dimension error! Quitting.')
            grid_grabbed = False
            continue

        # No errors, can proceed to inflation:
        obstacle_vec = np.array([map_data>GRID_THRESHOLD], dtype=np.int8).flatten()
        obstacle_mat = np.reshape(obstacle_vec, (grid_height, grid_width))
        for pad_level in range(NUM_CELLS_TO_PAD):
            pad_obst_mat = np.pad(obstacle_mat, pad_width=1, mode='constant', constant_values=0)
            sum_obst_mat = pad_obst_mat[1:grid_height+1, 1:grid_width+1]
            for i in [-1, 0, 1]:
                for j in [-1, 0, 1]:
                    sum_obst_mat = sum_obst_mat + pad_obst_mat[1+i:grid_height+1+i, 1+j:grid_width+1+j]

        scaled_obst_mat = np.array([sum_obst_mat>0], dtype=np.int8).flatten() * 100

        grid.data = list(scaled_obst_mat)
        grid.header.stamp = rospy.Time.now()
        pub.publish(grid)

        grid_grabbed = False # clear flag :)

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
