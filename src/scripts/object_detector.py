#!/usr/bin/env python2

from __future__ import print_function
import rospy
import sys
import os
import csv
import warnings
import traceback
import signal
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid

class SigIntException(BaseException): pass

def stop(signal, frame):
    rospy.loginfo("You pressed ctrl-c")
    raise SigIntException

scan_grid = OccupancyGrid()
obst_grid = OccupancyGrid()
done_grid = OccupancyGrid()
scan_got = False
obst_got = False
ego_pos = [0.0, 0.0]
GRID_THRESHOLD = 65 # over this is an obstacle :)

def callback_scan(msg):
    global scan_grid, scan_got
    scan_grid = msg
    scan_got = True

def callback_obst(msg):
    global obst_grid, obst_got
    if not obst_got:
        obst_grid = msg
        done_grid.info = obst_grid.info
        obst_got = True

def ros_node_main():
    global obst_got, scan_got, obst_grid, scan_grid, done_grid, GRID_THRESHOLD
    rospy.init_node('object_detector', anonymous=True)
    sub_scan = rospy.Subscriber('/scan2pc2grid_aligned', OccupancyGrid, callback_scan, queue_size=1)
    sub_obst = rospy.Subscriber('/map/validpath', OccupancyGrid, callback_obst, queue_size=1)
    pub_done = rospy.Publisher('/map/obstacles', OccupancyGrid, queue_size=1)
    rate = rospy.Rate(15)  # 15 [Hz]
    save_state = 0
    # do listen & save operation
    try:
        while not rospy.is_shutdown():
            if obst_got:
                if scan_got:
                    done_grid.header.stamp = rospy.Time.now()
                    done_grid.header.frame_id = scan_grid.header.frame_id
                    scan_slice = np.asarray(scan_grid.data, dtype=np.int8) # prep data for numpy
                    obst_slice = np.asarray(obst_grid.data, dtype=np.int8) # ^^
                    scan_binary = np.array([scan_slice > GRID_THRESHOLD], dtype=np.int8).flatten() # mask only obstacles
                    obst_binary = np.array([obst_slice == 0], dtype=np.int8).flatten() # mask only track
                    done_grid.data = (obst_binary*-1 + 1 + obst_binary * scan_binary ) * 100# apply mask to incoming data
                    pub_done.publish(done_grid)
                    scan_got = False
            try:
                rate.sleep()
            except:
                break

    except SigIntException:
        pass
    print("Quit received.")


if __name__ == '__main__':
    try:
        ros_node_main()
    except rospy.ROSInterruptException:
        pass


