#!/usr/bin/env python2

import rospy
import matplotlib.pyplot as plt
import numpy as np
import math
import csv
import os
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler

FRAME_ID = "map"

def path_plotter():
    global FRAME_ID
    rospy.init_node('path_publisher', anonymous=True)
    rate = rospy.Rate(5)  # 5 [Hz]
    wp_pub = rospy.Publisher('/points_done', Path, queue_size=1)
        
    rospy.loginfo('Started Path Plotter node.')

    x_data = []
    y_data = []
    file_path = os.path.join(os.path.dirname(__file__), 'points.csv')
    with open(file_path) as f:
        reader = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
        data = list(reader)
        x_data = data[0]
        y_data = data[1]
        f.close()

    # make map
    waypoint_map = Path() # re-init
    waypoint_map.header.seq = 0
    waypoint_map.header.stamp = rospy.Time.now()
    waypoint_map.header.frame_id = FRAME_ID
    waypoint_map.poses = []
    if len(x_data) > 1:
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

    # do publish operation
    while not rospy.is_shutdown():
        wp_pub.publish(waypoint_map)

        rate.sleep()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    plt.close('all')

if __name__ == '__main__':
    path_plotter()
