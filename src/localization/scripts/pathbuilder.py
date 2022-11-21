#!/usr/bin/env python2

import rospy
import matplotlib.pyplot as plt
import numpy as np
import math
import os
import csv
from std_msgs.msg import String, Int8, Bool, Float64MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion
from nav_msgs.msg import Path
from scipy.interpolate import splprep, splev
from tf.transformations import quaternion_from_euler

xpoints = []
ypoints = []
x_new = []
y_new = []
do_publish = True
map_made = False
do_draw = False
do_add = False
draw_change = False
FRAME_ID = "map"

def callback_pose(msg):
    global xpoints, ypoints, x_new, y_new, map_made, draw_change, do_add
    if not do_add:
        return
    draw_change = True
    map_made = False
    xpoints.append(msg.pose.pose.position.x)
    ypoints.append(msg.pose.pose.position.y)
    if len(xpoints) < 8:
        return

    sortedx = [xpoints[0]]
    sortedy = [ypoints[0]]
    xpoints.pop(0)
    ypoints.pop(0)
    for i in range(len(xpoints)):
        mags = np.power(np.add(np.square(np.array(xpoints)-sortedx[-1]), 
                               np.square(np.array(ypoints)-sortedy[-1])
                        ), 0.5)
        min_ind = np.argmin(mags)
        sortedx.append(xpoints[min_ind])
        sortedy.append(ypoints[min_ind])
        xpoints.pop(min_ind)
        ypoints.pop(min_ind)

    xpoints = sortedx
    ypoints = sortedy

    # https://stackoverflow.com/questions/31464345/fitting-a-closed-curve-to-a-set-of-points
    point_stack = np.stack((np.array(xpoints),np.array(ypoints)))
    tck, u = splprep(point_stack, u=None, s=0.5, per=1) 
    u_new = np.linspace(u.min(), u.max(), 400) # 400 points will give a resolution of 0.1 [m] for a track length of 40 [m] (ours is about 30 [m])
    x_new[:] = []
    y_new[:] = []
    x_new_np, y_new_np = splev(u_new, tck, der=0)
    x_new = list(x_new_np)
    y_new = list(y_new_np)

def callback_publ(msg):
    global do_publish
    do_publish = msg.data

def callback_plot(msg):
    global do_draw, draw_change
    do_draw = msg.data
    draw_change = True

def callback_addn(msg):
    global do_add
    do_add = msg.data

def callback_save(msg): # we don't use msg
    global x_new, y_new, xpoints, ypoints
    print("Saving points to points.csv.")
    file_path = os.path.join(os.path.dirname(__file__), 'points.csv')
    with open(file_path) as f:
        write = csv.writer(f)
        write.writerow(x_new)
        write.writerow(y_new)
        f.close()

def callback_load(msg): # we don't use msg
    global x_new, y_new, xpoints, ypoints, map_made, draw_change
    draw_change = True
    map_made = False
    print("Loading points from points.csv.")
    file_path = os.path.join(os.path.dirname(__file__), 'points.csv')
    with open(file_path) as f:
        reader = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
        data = list(reader)
        x_new[:] = []
        y_new[:] = []
        xpoints[:] = []
        ypoints[:] = []
        x_new = data[0]
        y_new = data[1]
        xpoints = data[0]
        ypoints = data[1]
        f.close()
    print("(%d,%d)" % (len(x_new), len(y_new)))

def callback_dele(msg): # we don't use msg
    global x_new, y_new, xpoints, ypoints, map_made, draw_change
    draw_change = True
    map_made = False
    print("Clearing all points.")
    x_new[:] = []
    y_new[:] = []
    xpoints[:] = []
    ypoints[:] = []

def pose_printer():
    global xpoints, ypoints, x_new, y_new, map_made, do_publish, FRAME_ID, do_draw, draw_change
    rospy.init_node('path_builder', anonymous=True)
    rate = rospy.Rate(5)  # 5 [Hz]
    wp_pub = rospy.Publisher('/points_done', Path, queue_size=10)

    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, callback_pose)
    rospy.Subscriber('/points_publ', Bool, callback_publ)
    rospy.Subscriber('/points_plot', Bool, callback_plot)
    rospy.Subscriber('/points_addn', Bool, callback_addn)
    rospy.Subscriber('/points_save', Int8, callback_save) # pure interrupt, msg content doesn't matter
    rospy.Subscriber('/points_load', Int8, callback_load) # pure interrupt, ^^
    rospy.Subscriber('/points_dele', Int8, callback_dele) # pure interrupt, ^^
        
    rospy.loginfo('Started Path Builder node.')

    waypoint_map = Path()
    try:
	f1 = plt.figure()
    	plt.ion()
    except:
	print("Plot error, caught and ignoring. Plotting likely disabled.")
    while not rospy.is_shutdown():
        if do_draw and draw_change:
            try:
		plt.show(block=False)
            	plt.clf()
            	if len(xpoints) > 7:
               		plt.plot(x_new, y_new, 'b--')
            	plt.plot(xpoints, ypoints, '.')
            	plt.title("%d Points" % len(xpoints))
            	axes = plt.axes()
            	axes.set_xlim([-8, 8])
            	axes.set_ylim([-8, 8])
            	plt.draw()
            	plt.pause(0.001)
	    except:
		print("Plot error, caught and ignoring. Plotting likely disabled.")
            draw_change = False
        elif not do_draw:
            plt.close('all')

        if do_publish:
            if not map_made:
                # make map
                waypoint_map = Path() # re-init
                waypoint_map.header.seq = 0
                waypoint_map.header.stamp = rospy.Time.now()
                waypoint_map.header.frame_id = FRAME_ID
                waypoint_map.poses = []
                if len(x_new) > 1:
                    for i in range(len(x_new)-1):
                        pose_to_add = PoseStamped()
                        pose_to_add.header.seq = i + 1
                        pose_to_add.header.stamp = waypoint_map.header.stamp
                        pose_to_add.header.frame_id = FRAME_ID

                        # set position:
                        pose_to_add.pose.position.x = x_new[i]
                        pose_to_add.pose.position.y = y_new[i]
                        pose_to_add.pose.position.z = 0.0

                        # generate orientation quaternion:
                        orientation_vec = [x_new[i+1]-x_new[i], y_new[i+1]-y_new[i], 0] 
                        # assume 0 roll and 0 pitch
                        q = quaternion_from_euler(0, 0, math.atan2(orientation_vec[1], orientation_vec[0])) # roll, pitch, yaw

                        # set orientation
                        pose_to_add.pose.orientation.x = q[0]
                        pose_to_add.pose.orientation.y = q[1]
                        pose_to_add.pose.orientation.z = q[2]
                        pose_to_add.pose.orientation.w = q[3]

                        waypoint_map.poses.append(pose_to_add)

                map_made = True # set flag
            # do publish operation
            wp_pub.publish(waypoint_map)

        rate.sleep()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    plt.close('all')

if __name__ == '__main__':
    pose_printer()
