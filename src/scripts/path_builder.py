#!/usr/bin/env python2

import rospy
import matplotlib.pyplot as plt
import numpy as np
import math
import os
import sys
import csv
import warnings
import traceback
from std_msgs.msg import String, Int8, Bool, Float64MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion
from nav_msgs.msg import Path
from scipy.interpolate import splprep, splev
from tf.transformations import quaternion_from_euler

x_sparse = []
y_sparse = []
x_dense = []
y_dense = []
do_publish = True
map_made = False
do_draw = False
do_add = True
draw_change = False
FRAME_ID = "map"
POINT_DENSITY = 0.05

class SigIntException(BaseException): pass

def stop(signal, frame):
    rospy.loginfo("You pressed ctrl-c")
    raise SigIntException

def callback_pose(msg):
    global x_sparse, y_sparse, x_dense, y_dense, map_made, draw_change, do_add
    if not do_add:
        return
    draw_change = True
    map_made = False
    x_process = []
    y_process = []

    try:
        x_process = list(x_sparse)
        y_process = list(y_sparse)
        x_process.append(msg.pose.pose.position.x)
        y_process.append(msg.pose.pose.position.y)
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

        # https://stackoverflow.com/questions/31464345/fitting-a-closed-curve-to-a-set-of-points
        point_stack = np.stack((np.array(sortedx), np.array(sortedy)))
        x_dense[:] = []
        y_dense[:] = []

        with warnings.catch_warnings():
            warnings.filterwarnings('ignore')#, r'/usr/lib/python2.7/dist-packages/scipy/interpolate/_fitpack_impl.py:227')
            tck, u = splprep(point_stack, u=None, s=1.0, per=1) 
            u_dense = np.linspace(u.min(), u.max(), 2000)
            x_dense_np, y_dense_np = splev(u_dense, tck, der=0)
            x_dense_list = list(x_dense_np)
            y_dense_list = list(y_dense_np)

            count = 1
            while count < len(x_dense_list):
                if np.power(np.square(x_dense_list[count] - x_dense_list[count - 1]) + \
                            np.square(y_dense_list[count] - y_dense_list[count - 1]) \
                            , 0.5) < POINT_DENSITY:
                    x_dense_list.pop(count)
                    y_dense_list.pop(count)
                    count = count - 1
                    print("killed")
                count = count + 1

        # if we get to here then we are safe :)
        x_dense = list(x_dense_list)
        y_dense = list(y_dense_list)
        x_sparse[:] = []
        y_sparse[:] = []
        x_sparse = list(sortedx)
        y_sparse = list(sortedy)

    except Exception as E:
        # https://www.adamsmith.haus/python/answers/how-to-retrieve-the-file,-line-number,-and-type-of-an-exception-in-python
        exception_type, exception_object, exception_traceback = sys.exc_info()
        #filename = exception_traceback.tb_frame.f_code.co_filename
        line_number = exception_traceback.tb_lineno
        print("Error: skipping point addition to preserve stack.")
        print("\tException type: " + str(E))
        #print("\tFile name: " + str(filename))
        print("\tLine number: " + str(line_number))

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
    global x_dense, y_dense, x_sparse, y_sparse
    print("Saving points to points.csv.")
    file_path = os.path.join(os.path.dirname(__file__), 'csv/points.csv')
    with open(file_path, "w") as f:
        write = csv.writer(f)
        write.writerow(x_sparse) # for loading and editing purposes
        write.writerow(y_sparse) # ^
        write.writerow(x_dense) # for plotting and path purposes
        write.writerow(y_dense) # ^
        f.close()

def callback_load(msg): # we don't use msg
    global x_dense, y_dense, x_sparse, y_sparse, map_made, draw_change
    draw_change = True
    map_made = False
    print("Loading points from points.csv.")
    file_path = os.path.join(os.path.dirname(__file__), 'csv/points.csv')

    with open(file_path) as f:
        reader = csv.reader(f, quoting=csv.QUOTE_NONNUMERIC)
        data = list(reader)
        x_sparse[:] = []
        y_sparse[:] = []
        x_dense[:] = []
        y_dense[:] = []
        x_sparse = data[0]
        y_sparse = data[1]
        x_dense = data[2]
        y_dense = data[3]
        f.close()
    print("(%d,%d)" % (len(x_dense), len(y_dense)))

def callback_dele(msg): # we don't use msg
    global x_dense, y_dense, x_sparse, y_sparse, map_made, draw_change
    draw_change = True
    map_made = False
    print("Clearing all points.")
    x_dense[:] = []
    y_dense[:] = []
    x_sparse[:] = []
    y_sparse[:] = []

def pose_printer():
    global x_sparse, y_sparse, x_dense, y_dense, map_made, do_publish, FRAME_ID, do_draw, draw_change
    rospy.init_node('path_builder', anonymous=True, disable_signals=True)
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
    try:
        while not rospy.is_shutdown():
            if do_draw and draw_change:
                try:
                    plt.show(block=False)
                    plt.clf()
                    if len(x_sparse) > 7:
                        plt.plot(x_dense, y_dense, 'b--')
                    plt.plot(x_sparse, y_sparse, '.')
                    plt.title("%d Points" % len(x_sparse))
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
                    if len(x_dense) > 1:
                        for i in range(len(x_dense)-1):
                            pose_to_add = PoseStamped()
                            pose_to_add.header.seq = i + 1
                            pose_to_add.header.stamp = waypoint_map.header.stamp
                            pose_to_add.header.frame_id = FRAME_ID

                            # set position:
                            pose_to_add.pose.position.x = x_dense[i]
                            pose_to_add.pose.position.y = y_dense[i]
                            pose_to_add.pose.position.z = 0.0

                            # generate orientation quaternion:
                            orientation_vec = [x_dense[i+1]-x_dense[i], y_dense[i+1]-y_dense[i], 0] 
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
            try:
                rate.sleep()
            except:
                break
    except SigIntException:
        pass
    print("Quit received.")
    plt.close('all')

if __name__ == '__main__':
    pose_printer()
