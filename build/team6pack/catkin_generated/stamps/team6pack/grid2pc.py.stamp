#!/usr/bin/env python2

from __future__ import print_function
import rospy
import math
import numpy as np
import sys
import signal

from sensor_msgs import point_cloud2
import laser_geometry.laser_geometry as lg

from sensor_msgs.msg import PointCloud2, PointField, LaserScan
from std_msgs.msg import Header, Time
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion

# global variables because we're lazy
grid = OccupancyGrid()
map_origin = Pose()
map_time = Time()
grid_grabbed = False
pc_ready = False

GRID_THRESHOLD = 65 # over this is an obstacle :)
MAP_TOPIC = '/map'
PC_TOPIC = '/grid2pc'

# https://stackoverflow.com/questions/40438261/using-sigint-to-kill-a-function-in-python-3
class SigIntException(BaseException): pass

def stop(signal, frame):
    print("You pressed ctrl-c")
    raise SigIntException

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
    global grid, grid_grabbed, map_origin, map_time, GRID_THRESHOLD, MAP_TOPIC, pc_ready

    rospy.init_node('grid2pc_node', anonymous=True)
    rate = rospy.Rate(rate) # 1 Hz by default
    #pc_sub = rospy.Subscriber('/scan2pc', PointCloud2, scan_cb, queue_size=1)
    om_sub = rospy.Subscriber(MAP_TOPIC, OccupancyGrid, grid_cb, queue_size=1)
    pc_pub = rospy.Publisher(PC_TOPIC, PointCloud2, queue_size=1)
    rospy.loginfo('Converting grid to pointcloud.')

    # https://gist.github.com/lucasw/ea04dcd65bc944daea07612314d114bb

    fields = [  PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)
             ] # just x, y, z

    header = Header()
    header.frame_id = "map"
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
                        ix = np.tile( np.array(range(0,grid_width,1), dtype=float), (1, grid_height)).flatten()
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

                      # Method B: !! OLD, really slow :(  !!
                #     map_data_premask = np.asarray(grid.data, dtype=np.int8).reshape(grid_height, grid_width)
                #     map_data = np.ma.array(map_data_premask, mask=map_data_premask==-1, fill_value=-1)
                
                #     point = [0.0, 0.0, 0.0] # init as float to be safe :)
                #     rospy.loginfo('Map dimensions: %ix%i [%i cells].' % (grid_width, grid_height, grid_width*grid_height))
                #     for x_ind in range(grid_width):
                #         for y_ind in range(grid_height): 
                #             if (map_data[y_ind][x_ind] > GRID_THRESHOLD):
                #                 point = [x_ind * grid_resolution + map_origin.position.x, y_ind * grid_resolution + map_origin.position.y, 0]
                #                 points.append(point)
                #                 #rospy.loginfo("\t DEBUG: %f,%f,%f" % (point[0],point[1],point[2]))
                    if (len(points) > 0):
                        rospy.loginfo('Map generated! Commencing publish to ' + PC_TOPIC + '...')           
                        pc = point_cloud2.create_cloud(header, fields, points)
                        pc.header.stamp = rospy.Time.now()
                        pc_ready = True
                    else:
                        rospy.loginfo('Map generated is unoccupied! Skipping.')
                else:
                    rospy.loginfo('Map received has null dimension! Skipping.')

                
                #rospy.loginfo('rid dims: %i, %i' % (grid_width, grid_height))
                grid_grabbed = False # clear flag :)
            if pc_ready:
                pc_pub.publish(pc)

            rate.sleep()
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

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
