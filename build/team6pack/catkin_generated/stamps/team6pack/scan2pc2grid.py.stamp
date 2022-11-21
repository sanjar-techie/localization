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

# global variables for access
MAP_RESOLUTION = 0.05
scan_msg = LaserScan()
new_msg = False

class SigIntException(BaseException): pass

def stop(signal, frame):
    print("You pressed ctrl-c")
    raise SigIntException

def scan_cb(msg):
    global scan_msg, new_msg
    scan_msg = msg
    new_msg = True

def ros_node_main():
    global scan_msg, new_msg, MAP_RESOLUTION
    rospy.init_node('scan2pc2grid_node', anonymous=True)
    rate = rospy.Rate(40) # 40 Hz
    lp = lg.LaserProjection()
    pc_pub = rospy.Publisher("/scan2pc", PointCloud2, queue_size=1)
    og_pub = rospy.Publisher("/scan2pc2grid", OccupancyGrid, queue_size=1)
    ls_sub = rospy.Subscriber('/scan', LaserScan, scan_cb, queue_size=1)
    rospy.loginfo('Converting laserscan to pointcloud and then to grid.')

    try:
        while not rospy.is_shutdown():
            if (new_msg):
                # convert the message of type LaserScan to a PointCloud2
                pc2_msg = lp.projectLaser(scan_msg)
                assert isinstance(pc2_msg, PointCloud2)
                pc_pub.publish(pc2_msg)

                # convert from PointCloud2 to OccupancyGrid steps ...
                # read in point_cloud2 as an accessible numpy array:
                gen = np.array(list(point_cloud2.read_points(pc2_msg, skip_nans=True, field_names=("x", "y", "z"))))

                # delete z values (all will be zero regardless):
                genxy = np.delete(gen, 2, axis=1)

                # round to nearest multiple of MAP_RESOLUTION:
                genxyfloat = np.array(MAP_RESOLUTION * np.around(genxy/MAP_RESOLUTION, 0))

                # remove duplicate point entries (only unique rows; parse through integer to avoid floating point equalities):
                genxyfloatclean = np.array(np.unique(np.around(genxyfloat*100,0), axis=0)/100.0,dtype=float)

                # calculate min, max, and range of dataset ([0] is x, [1] is y):
                gen_mins = np.amin(genxyfloatclean, axis=0) # must be [0, 0]
                gen_maxs = np.amax(genxyfloatclean, axis=0)
                x_range = abs(gen_maxs[0] - gen_mins[0]) + 2
                y_range = abs(gen_maxs[1] - gen_mins[1]) + 2
                map_width = int(x_range/MAP_RESOLUTION) # number of x indices
                map_height = int(y_range/MAP_RESOLUTION) # number of y indices

                # build empty map (note; number of rows -> map_height)
                empty_grid = np.ones((map_height, map_width)) * -1

                # populate map with pointcloud at correct cartesian coordinates
                for i in range(np.shape(genxyfloatclean)[0]):
                    iyp = genxyfloatclean[i,1] - gen_mins[1] + 1
                    ixp = genxyfloatclean[i,0] - gen_mins[0] + 1

                    iy = int( iyp / MAP_RESOLUTION ) + 1
                    ix = int( ixp / MAP_RESOLUTION ) + 1

                    empty_grid[iy][ix] = 100

                # convert to (u)int8 for ROS message transmission
                populated_grid = np.array(empty_grid, dtype=np.int8).flatten()
             
                # the rest is building the OccupancyGrid() object and populating it for transmission
                grid = OccupancyGrid()
                grid_header = Header()
                grid_header.frame_id = "map"
                grid_header.stamp = rospy.Time.now()

                grid_mma_pose = Pose()
                grid_mma_pose.position.x = gen_mins[0] - 1.05
                grid_mma_pose.position.y = gen_mins[1] - 1.05
                grid_mma_pose.position.z = 0
                grid_mma_pose.orientation.x = 0.0
                grid_mma_pose.orientation.y = 0.0
                grid_mma_pose.orientation.z = 0.0
                grid_mma_pose.orientation.w = 0.0

                grid_mma = MapMetaData()
                grid_mma.map_load_time = rospy.Time.now()
                grid_mma.width = map_width 
                grid_mma.height = map_height
                grid_mma.origin = grid_mma_pose
                grid_mma.resolution = MAP_RESOLUTION

                grid.info = grid_mma
                grid.header = grid_header
                grid.data = populated_grid

                og_pub.publish(grid)
                new_msg = False
            rate.sleep()
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    except SigIntException:
        rospy.loginfo('Quit received.')


if __name__ == '__main__':
    ros_node_main()
