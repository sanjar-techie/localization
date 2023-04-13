#!/usr/bin/env python2

from __future__ import print_function
import rospy
import math
import numpy as np
import sys
import os
import csv
import signal

from sensor_msgs import point_cloud2
import laser_geometry.laser_geometry as lg

from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from std_msgs.msg import Header, Time, Int8
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry, Path
from geometry_msgs.msg import Pose, Twist, Quaternion, PoseStamped

from tf.transformations import quaternion_from_euler

# global variables because we're lazy
grid = OccupancyGrid()
map_origin = Pose()
map_time = Time()
grid_grabbed = False
msg_ready = False

GRID_THRESHOLD = 65 # over this is an obstacle :)
MAP_TOPIC = '/map/truepath'
PATH_TOPIC = '/path'
FRAME_ID = "map"
POINT_DENSITY = 0.05

x_data = []
y_data = []

# https://stackoverflow.com/questions/40438261/using-sigint-to-kill-a-function-in-python-3
class SigIntException(BaseException): pass

def stop(signal, frame):
    print("You pressed ctrl-c")
    raise SigIntException

def grid_cb(msg):
    global grid, grid_grabbed, MAP_TOPIC, msg_ready
    try:
        grid = msg
        grid_grabbed = True
        msg_ready = False
        #rospy.loginfo('Grid received on ' + MAP_TOPIC + '! Stored.')
    except SigIntException:
            return

def callback_save(msg): # we don't use msg
    global x_data, y_data
    print("Saving points to path.csv.")
    file_path = os.path.join(os.path.dirname(__file__), 'csv/path.csv')
    with open(file_path, "w") as f:
        f.truncate()
        write = csv.writer(f)
        write.writerow([]) # for loading and editing purposes
        write.writerow([]) # ^
        write.writerow(x_data) # for plotting and path purposes
        write.writerow(y_data) # ^
        f.close()

def ros_node_main(rate):
    global grid, grid_grabbed, map_origin, map_time, POINT_DENSITY, GRID_THRESHOLD, MAP_TOPIC, msg_ready, FRAME_ID, x_data, y_data

    rospy.init_node('grid2pc_node', anonymous=True, disable_signals=True)
    rate = rospy.Rate(rate) # 1 Hz by default
    #pc_sub = rospy.Subscriber('/scan2pc', PointCloud2, scan_cb, queue_size=1)
    om_sub = rospy.Subscriber(MAP_TOPIC, OccupancyGrid, grid_cb, queue_size=1)
    path_pub = rospy.Publisher(PATH_TOPIC, Path, queue_size=1)
    rospy.Subscriber('/points_save', Int8, callback_save)
    rospy.loginfo('Converting grid to pointcloud.')

    # https://gist.github.com/lucasw/ea04dcd65bc944daea07612314d114bb

    fields = [  PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)
             ] # just x, y, z

    header = Header()
    header.frame_id = FRAME_ID
    try:
        while not rospy.is_shutdown():
            if (grid_grabbed): # grid has been received!
                # http://docs.ros.org/en/jade/api/ros_numpy/html/occupancy__grid_8py_source.html 
                # make grid variables:
                grid_width = grid.info.width
                grid_height = grid.info.height
                grid_resolution = grid.info.resolution
                map_origin = grid.info.origin
                map_time = grid.info.map_load_time

                points = [] # init empty
                if (grid_width > 0 and grid_height > 0):
                    # Method A:
                    map_data_premask = np.asarray(grid.data, dtype=np.int8)
                    map_data = np.ma.array(map_data_premask, mask=map_data_premask==-1, fill_value=-1)
                    
                    if (grid_width * grid_height == len(map_data)): # dim is okay!
                        # create indices row matrices and final stacked matrix
                        ix = np.tile( np.array(range(grid_width), dtype=float), (1, grid_height)).flatten()
                        iy = np.transpose(np.tile( np.array(range(0,grid_height,1), dtype=float), (grid_width, 1))).flatten()
                        iz = np.zeros(grid_width*grid_height, dtype=float)
                        indices_mat = np.concatenate(([ix],[iy],[iz]), axis=0)

                        # filter indices matrix by occupancygrid threshold
                        filter_mat = ~np.array([map_data>GRID_THRESHOLD]).flatten() #not'd to get correct filt
                        column_mat = np.array(range(grid_width*grid_height))
                        cols_filt = column_mat[filter_mat]
                        filtered_map = np.delete(indices_mat, cols_filt, axis=1)

                        # scale each point to the resolution, offset by origin, and convert to list
                        scaled_map = filtered_map * grid_resolution
                        translation_vec = np.array([map_origin.position.x, map_origin.position.y, 0])
                        translation_mat = np.transpose(np.tile(translation_vec, (np.shape(scaled_map)[1], 1)))

                        scaled_translated_map = np.add(scaled_map, translation_mat)
                        points = list(scaled_translated_map.T)
                    else:
                        rospy.loginfo('Dimension error! Quitting.')

                    if (np.shape(scaled_translated_map[0][:]) > 0):

                        x_data[:] = []
                        y_data[:] = []
                        x_data = list(scaled_translated_map[0][:])
                        y_data = list(scaled_translated_map[1][:])

                        # sort data:
                        x_process = list(x_data)
                        y_process = list(y_data)

                        if len(x_process) < 8:
                            x_sparse = list(x_process)
                            y_sparse = list(y_process)
                            return
                        sortedx = list([x_process[0]])
                        sortedy = list([y_process[0]])
                        x_process.pop(0)
                        y_process.pop(0)
                        for i in range(len(x_process)):
                            mags = np.power(np.add(np.square(np.array(x_process)-sortedx[-1]), 
                                                   np.square(np.array(y_process)-sortedy[-1])
                                            ), 0.5)
                            min_ind = np.argmin(mags)
                            sortedx.append(x_process[min_ind])
                            sortedy.append(y_process[min_ind])
                            x_process.pop(min_ind)
                            y_process.pop(min_ind)

                        count = 1
                        while count < len(sortedx):
                            if np.power(np.square(sortedx[count] - sortedx[count - 1]) + \
                                        np.square(sortedy[count] - sortedy[count - 1]) \
                                        , 0.5) < POINT_DENSITY:
                                sortedx.pop(count)
                                sortedy.pop(count)
                                count = count - 1
                            count = count + 1

                        x_data[:] = []
                        y_data[:] = []
                        x_data = list(sortedx)
                        y_data = list(sortedy)

                        # make map
                        waypoint_map = Path() # re-init
                        waypoint_map.header.seq = 0
                        waypoint_map.header.stamp = rospy.Time.now()
                        waypoint_map.header.frame_id = FRAME_ID
                        waypoint_map.poses = []

                        for i in range(len(x_data)-1):
                            pose_to_add = PoseStamped()
                            pose_to_add.header.seq = i + 1
                            pose_to_add.header.stamp = waypoint_map.header.stamp
                            pose_to_add.header.frame_id = FRAME_ID

                            # set position:
                            pose_to_add.pose.position.x = x_data[i]
                            pose_to_add.pose.position.y = y_data[i]
                            pose_to_add.pose.position.z = 0.0

                            # generate orientation quaternion:
                            orientation_vec = [x_data[i+1]-x_data[i], y_data[i+1]-y_data[i], 0] 
                            # assume 0 roll and 0 pitch
                            q = quaternion_from_euler(0, 0, math.atan2(orientation_vec[1], orientation_vec[0])) # roll, pitch, yaw

                            # set orientation
                            pose_to_add.pose.orientation.x = q[0]
                            pose_to_add.pose.orientation.y = q[1]
                            pose_to_add.pose.orientation.z = q[2]
                            pose_to_add.pose.orientation.w = q[3]

                            waypoint_map.poses.append(pose_to_add)

                        rospy.loginfo('PATH generated! Commencing publish to ' + PATH_TOPIC + '...')           
                        msg_ready = True
                    else:
                        rospy.loginfo('Map generated is unoccupied! Skipping.')
                else:
                    rospy.loginfo('Map received has null dimension! Skipping.')

                #rospy.loginfo('rid dims: %i, %i' % (grid_width, grid_height))
                grid_grabbed = False # clear flag :)
            if msg_ready:
                path_pub.publish(waypoint_map)
            try:
                rate.sleep()
            except:
                break
    except SigIntException:
        pass
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
