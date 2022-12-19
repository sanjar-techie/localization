#!/usr/bin/env python2

from __future__ import print_function
import rospy
import math
import time

from sensor_msgs import point_cloud2
import laser_geometry.laser_geometry as lg

from sensor_msgs.msg import PointCloud2, LaserScan

# global variables for access
scan_msg = LaserScan()
new_msg = False
pc_msg = PointCloud2()

def scan_cb(msg):
    global scan_msg, new_msg
    scan_msg = msg
    new_msg = True

def ros_node_main():
    global scan_msg, new_msg, pc_msg
    rospy.init_node('scan2pc_node', anonymous=True)
    rate = rospy.Rate(40) # 40 Hz
    lp = lg.LaserProjection()
    pc_pub = rospy.Publisher("/scan2pc", PointCloud2, queue_size=1)
    ls_sub = rospy.Subscriber('/scan', LaserScan, scan_cb, queue_size=1)

    rospy.loginfo('Converting laserscan to pointcloud.')
    #st = time.time()
    while not rospy.is_shutdown():
        #et = time.time()
        #period = et - st
        #print((period, 1.0/period))
        #st = time.time()

        if (new_msg):
            # convert the message of type LaserScan to a PointCloud2
            new_pc_msg = lp.projectLaser(scan_msg)
            assert isinstance(new_pc_msg, PointCloud2)
            pc_msg = new_pc_msg
            new_msg = False

        pc_pub.publish(pc_msg)

        rate.sleep()

    rospy.loginfo('Quit received.')


if __name__ == '__main__':
    ros_node_main()
