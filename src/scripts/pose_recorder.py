#!/usr/bin/env python2
# Software License Agreement (BSD License)

import rospy
import sys
import os
import csv
import warnings
import traceback
import numpy as np
from std_msgs.msg import String, Float32
from sensor_msgs.msg import PointCloud, ChannelFloat32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32

do_pub = False
x_points = []
y_points = []
point_cloud = PointCloud()
default_channel_details = ChannelFloat32()
default_channel_details.name = "intensity"
DIST_THRES = 0.04

class SigIntException(BaseException): pass

def stop(signal, frame):
    rospy.loginfo("You pressed ctrl-c")
    raise SigIntException

def callback_path(msg):
    global x_points, y_points, point_cloud, DIST_THRES, do_pub
    do_pub = False
    new_x = msg.pose.pose.position.x
    new_y = msg.pose.pose.position.y
    if len(x_points) == 0: 
        x_points.append(new_x)
        y_points.append(new_y)
    elif np.sqrt((new_x - x_points[-1])**2 + (new_y - y_points[-1])**2) > DIST_THRES:
        # avoid accessing empty list element
        x_points.append(new_x)
        y_points.append(new_y)

    # add to pc object
    new_point = Point32()
    new_point.x = new_x
    new_point.y = new_y
    new_point.z = 0
    ind = len(point_cloud.points) 

    point_cloud.points.append(new_point)
    point_cloud.channels[0].values.append(1.0)
    do_pub = True

def ros_node_main(fname):
    global x_points, y_points, point_cloud, default_channel_details
    rospy.init_node('pose_recorder', anonymous=True)
    sub_odom = rospy.Subscriber('/odometry/filtered', Odometry, callback_path)
    pub_pocl = rospy.Publisher('/pose_recorder/pointcloud', PointCloud, queue_size=1)
    rate = rospy.Rate(15)  # 15 [Hz]

    # initialise pc object
    point_cloud.header.frame_id = "map"
    point_cloud.channels.append(default_channel_details)

    save_state = 0
    # do listen & save operation
    try:
        while not rospy.is_shutdown():
            if not rospy.has_param('/pose_recorder/save'):
                rospy.set_param('/pose_recorder/save', save_state) # set default direction
            else:
                save_state = rospy.get_param('/pose_recorder/save')

            if (save_state == 1):
                print("Saving points.")
                file_path = os.path.join(os.path.dirname(__file__), fname)
                with open(file_path, "w") as f:
                    write = csv.writer(f)
                    write.writerow([]) # empty for compatibility with path_builder & path_printer
                    write.writerow([]) # empty ^^
                    write.writerow(x_points)
                    write.writerow(y_points)
                    f.close()
                save_state = 0
                rospy.set_param('/pose_recorder/save', save_state)

            if len(x_points) > 0 and do_pub:
                point_cloud.header.stamp = rospy.Time.now()
                try:
                    pub_pocl.publish(point_cloud)
                except Exception as e:
                    print(e)
                    pass

            try:
                rate.sleep()
            except:
                break

    except SigIntException:
        pass
    print("Quit received.")


if __name__ == '__main__':
    try:
        filepath = "csv/border.csv"
        for argv_element in sys.argv:
            try: 
                val = str(argv_element)
                if os.path.abspath(__file__) == val: # don't return script path
                    continue
            except ValueError:
                continue
            filepath = str(argv_element)
            break
        ros_node_main(filepath)
    except rospy.ROSInterruptException:
        pass


