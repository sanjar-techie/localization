#!/usr/bin/env python2

import rospy
import numpy as np
import math
import csv
import os
import sys
import warnings
import traceback
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler

FRAME_ID = "map"

class SigIntException(BaseException): pass

def stop(signal, frame):
    rospy.loginfo("You pressed ctrl-c")
    raise SigIntException

def ros_node_main(fname):
    global FRAME_ID
    direction = 1
    rospy.init_node('path_printer', anonymous=True, disable_signals=True)
    rate = rospy.Rate(5)  # 5 [Hz]
    wp_pub = rospy.Publisher('/points_done', Path, queue_size=1)
        
    rospy.loginfo('Started Path Printer node.')

    x_data = []
    y_data = []
    file_path = os.path.join(os.path.dirname(__file__), fname)
    try:
        with open(file_path) as f:
            reader = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
            data = list(reader)
            x_data = data[2]
            y_data = data[3]
            f.close()
    except Exception as e:
        print("File does not exist or incorrect format. Quitting...")
        print(e)
        return

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

    #Reversed map
    waypoint_map_reversed = Path() # re-init
    waypoint_map_reversed.header.seq = 0
    waypoint_map_reversed.header.stamp = rospy.Time.now()
    waypoint_map_reversed.header.frame_id = FRAME_ID
    waypoint_map_reversed.poses = []
    if len(x_data) > 1:
        x_data_reversed = list(x_data)
        y_data_reversed = list(y_data)
        x_data_reversed.reverse()
        y_data_reversed.reverse()
        for i in range(len(x_data)-1):
            pose_to_add = PoseStamped()
            pose_to_add.header.seq = i + 1
            pose_to_add.header.stamp = waypoint_map_reversed.header.stamp
            pose_to_add.header.frame_id = FRAME_ID

            # set position:
            pose_to_add.pose.position.x = x_data_reversed[i]
            pose_to_add.pose.position.y = y_data_reversed[i]
            pose_to_add.pose.position.z = 0.0

            # generate orientation quaternion:
            orientation_vec = [x_data_reversed[i+1]-x_data_reversed[i], y_data_reversed[i+1]-y_data_reversed[i], 0] 
            # assume 0 roll and 0 pitch
            q = quaternion_from_euler(0, 0, math.atan2(orientation_vec[1], orientation_vec[0])) # roll, pitch, yaw

            # set orientation
            pose_to_add.pose.orientation.x = q[0]
            pose_to_add.pose.orientation.y = q[1]
            pose_to_add.pose.orientation.z = q[2]
            pose_to_add.pose.orientation.w = q[3]

            waypoint_map_reversed.poses.append(pose_to_add)

    # do publish operation
    try:
        while not rospy.is_shutdown():
            if not rospy.has_param('/controller/direction'):
                rospy.set_param('/controller/direction', direction) # set default direction
            else:
                direction = rospy.get_param('/controller/direction')
            
            if (direction == 1):
                wp_pub.publish(waypoint_map)
            else:
                wp_pub.publish(waypoint_map_reversed)

            try:
                rate.sleep()
            except:
                break

    except SigIntException:
        pass
    print("Quit received.")

if __name__ == '__main__':
    try:
        filepath = "csv/points.csv"
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
