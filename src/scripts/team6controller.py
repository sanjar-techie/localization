#!/usr/bin/env python2

from __future__ import print_function
import numpy as np
import os
import pandas as pd
import math
import signal
import sys
import time

import roslib
import rospy
import rospkg

from std_msgs.msg import Int16, Bool
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TwistStamped, PoseStamped

import tf
from tf.transformations import euler_from_quaternion

# minimise floating point wackies
def cutVal(toCut, threshold):
    if abs(toCut) < threshold:
        return float(0.0)
    return float(toCut)

# Normalize angle [-pi, +pi]
def normalize_angle(angle):
    if angle > math.pi:
        norm_angle = angle - 2*math.pi
    elif angle < -math.pi:
        norm_angle = angle + 2*math.pi
    else:
        norm_angle = angle
    return norm_angle

# Global2Local
def global2local(ego_x, ego_y, ego_yaw, x_list, y_list):
    Tx = x_list - ego_x
    Ty = y_list - ego_y
    R = np.sqrt(np.power(Tx, 2) + np.power(Ty, 2))
    A = np.arctan2(Ty, Tx) - ego_yaw

    output_x_list = np.multiply(np.cos(A), R) # DONE
    output_y_list = np.multiply(np.sin(A), R) # DONE

    return output_x_list, output_y_list

# Find nearest point
def find_nearest_point(ego_x, ego_y, x_list, y_list):
    distances = np.zeros(np.size(x_list))

    distances = np.sqrt(np.square(x_list - ego_x) + np.square(y_list - ego_y))

    near_ind = np.argmin(distances) # DONE
    near_dist = distances[near_ind] # DONE

    return near_dist, near_ind

class WaypointFollower():
    def __init__(self):
        # ROS init
        rospy.init_node('team6controller_node', anonymous=True, disable_signals=True)
        self.rate = rospy.Rate(40.0)

        # Params
        self.MAX_SPEED = 0.5#10.0/3.6
        self.MAX_STEER = np.deg2rad(15)

        self.MIN_PWM = 1100 
        self.MAX_PWM = 1900

        # vehicle state
        self.ego_x   = 0.0
        self.ego_y   = 0.0
        self.ego_yaw = 0.0
        self.ego_vx  = 0.0

        # histories
        self.error_yaw_history  = []
        self.error_y_history    = []
        self.error_v_history    = []

        self.steer_history      = []
        self.speed_history      = []

        self.ego_x_history      = []
        self.ego_y_history      = []
        self.ego_yaw_history    = []
        self.history_size       = 10

        # self.error_yaw_average = 0.0
        # self.error_y_average = 0.0
        # self.error_v_average = 0.0
        # self.average_measurements = 0.0

        self.gains_steer = [-1.0, 0.0, 0.5]
        self.gains_speed = [0.20, 0.0, 0.05]
        self.direction = 1

        self.wpt_look_ahead = 12   # [index]

        self.path_ready = 0
        self.wpts_x = []
        self.wpts_y = []
        self.wpts_poses = []
        self.next_index = 0

        # Pub/Sub
        self.pub_pwm_str = rospy.Publisher('/auto_cmd/steer', Int16, queue_size=1)
        self.pub_pwm_spd = rospy.Publisher('/auto_cmd/b_raw_throttle', Int16, queue_size=1) # actually publishing a speed...
        """
        publishing to b_raw_throttle, then using 
        "rosrun team6pack speed_pwm"
        to pulse the /auto_cmd/throttle at a specific duty cycle.
        This lets us access "slower speeds" without the motor cutting out (rather, we deliberately cut it out
        at a fixed rate, defined by parameters percent and freq). 
        """
        self.pub_car_mde        = rospy.Publisher('/auto_mode', Bool, queue_size=1, latch=True)
        self.pub_target_pose    = rospy.Publisher('/target_pose', PoseStamped, queue_size=1, latch=True)
        self.pub_cmd_vel        = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=1)
        self.sub_odom           = rospy.Subscriber('/odometry/filtered', Odometry, self.callback_odom, queue_size=1)
        self.sub_path           = rospy.Subscriber('/points_done', Path, self.callback_path, queue_size=1)
        self.sub_kill           = rospy.Subscriber('/car/kill', Bool, self.callback_kill, queue_size=1)

        self.car_mode = Bool()
        self.car_mode.data = True
        self.pub_car_mde.publish(self.car_mode)

    def callback_kill(self, msg):
        if msg.data:
            raise SigIntException
        rospy.loginfo("hello")

    def callback_path(self, msg):
        if self.path_ready == 0:
            self.wpts_x[:] = []
            self.wpts_y[:] = []
            self.wpts_poses[:] = []
            for pose_stamped in msg.poses:
                self.wpts_poses.append(pose_stamped)
                self.wpts_x.append(pose_stamped.pose.position.x)
                self.wpts_y.append(pose_stamped.pose.position.y)
            rospy.loginfo("Path received.")
            self.path_ready = 1
        return

    def callback_odom(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_vx = msg.twist.twist.linear.x
        
        q = msg.pose.pose.orientation
        q_list = [float(q.x), float(q.y), float(q.z), float(q.w)]
        _, _, ego_yaw_raw = euler_from_quaternion(q_list) # get euler from quaternion
        self.ego_yaw = float(ego_yaw_raw)

    def do_pid(self, value, hvalue, t, gains):
        value = cutVal(value, 0.00001)
        if len(hvalue):
            output = value*gains[0] + ((value-hvalue[-1])/t)*gains[1] + (sum(hvalue)+value)*t*gains[2]
        else:
            output = value*gains[0]
        return cutVal(output, 0.00001)

    def steer_control(self, error_yaw, error_y):
        steer_pre = self.do_pid(error_y, self.error_y_history, 1/100.0, self.gains_steer)
        steer = np.clip(steer_pre, -self.MAX_STEER, self.MAX_STEER) # Control limit
        return steer

    def speed_control(self, error_v):
        throttle = self.do_pid(error_v, self.error_v_history, 1/100.0, self.gains_speed)   
        throttle = np.clip(throttle, -self.MAX_SPEED, self.MAX_SPEED) # Control limit
        return throttle

    def update_history_front(self, new_x, new_y, new_yaw):
        while len(self.ego_x_history) >= self.history_size: # test for one as same for all
            self.ego_x_history.pop(0)
            self.ego_y_history.pop(0)
            self.ego_yaw_history.pop(0)

        self.ego_x_history.append(new_x)
        self.ego_y_history.append(new_y)
        self.ego_yaw_history.append(new_yaw)

    def update_history_mid(self, steer, speed):
        while len(self.steer_history) >= self.history_size: # test for one as same for all
            self.steer_history.pop(0)
            self.speed_history.pop(0)

        self.steer_history.append(steer)
        self.speed_history.append(speed)

    def update_history_end(self, new_eyaw, new_ey, new_ev):
        while len(self.error_yaw_history) >= self.history_size: # test for one as same for all
            self.error_yaw_history.pop(0)
            self.error_y_history.pop(0)
            self.error_v_history.pop(0)

        self.error_yaw_history.append(new_eyaw)
        self.error_y_history.append(new_ey)
        self.error_v_history.append(new_ev)

        #self.error_yaw_average = ((self.error_yaw_average * self.average_measurements) + new_yaw) / (self.average_measurements + 1)
        #self.error_y_average = ((self.error_y_average * self.average_measurements) + new_y) / (self.average_measurements + 1)
        #self.error_v_average = ((self.error_v_average * self.average_measurements) + new_v) / (self.average_measurements + 1)
        #self.average_measurements = self.average_measurements + 1

    def publish_command(self, steer, speed):
        """
        Publish command as Int16
        ref: http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Int16.html
        """
        raw_steer  = (((steer - -self.MAX_STEER)/(2 * self.MAX_STEER)) * 800.0) + 1100 # convert to PWM
        raw_speed  = (((speed - -self.MAX_SPEED)/(2 * self.MAX_SPEED)) * 800.0) + 1100 # ^^

        self.update_history_mid(int(round(raw_steer)), int(round(raw_speed)))

        msg_steer       = Int16()
        msg_speed       = Int16()

        msg_steer.data  = int(round(sum(self.steer_history)/len(self.steer_history)))
        msg_speed.data  = int(round(sum(self.speed_history)/len(self.speed_history)))

        twist_msg       = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        twist_msg.header.frame_id = "base_link"
        twist_msg.twist.linear.x = speed
        twist_msg.twist.angular.x = steer

        self.pub_cmd_vel.publish(twist_msg)
        self.pub_pwm_str.publish(msg_steer)
        self.pub_pwm_spd.publish(msg_speed)

    def do_params(self):
        if not rospy.has_param('/controller/gains_steer'):
            rospy.set_param('/controller/gains_steer', self.gains_steer) # set default gains
        else:
            self.gains_steer = rospy.get_param('/controller/gains_steer')

        if not rospy.has_param('/controller/gains_speed'):
            rospy.set_param('/controller/gains_speed', self.gains_speed) # set default gains
        else:
            self.gains_speed = rospy.get_param('/controller/gains_speed')

        if not rospy.has_param('/controller/history_size'):
            rospy.set_param('/controller/history_size', self.history_size) # set default history size
        else:
            self.history_size = rospy.get_param('/controller/history_size')

        if not rospy.has_param('/controller/direction'):
            rospy.set_param('/controller/direction', self.direction) # set default direction
        else:
            server_direction = rospy.get_param('/controller/direction')
            if server_direction != self.direction:
                self.path_ready = 0
                self.direction = server_direction


    def calc_error(self, ego_x, ego_y, ego_yaw, x_list, y_list, wpt_look_ahead=0): # Calculate Error
        # 1. Global to Local coordinate
        local_x_list, local_y_list = global2local(ego_x, ego_y, ego_yaw, x_list, y_list)

        # 2. Find the nearest waypoint
        _, near_ind = find_nearest_point(ego_x, ego_y, x_list, y_list)

        # 3. Set lookahead waypoint (index of waypoint trajectory)
        lookahead_wpt_ind = (near_ind + wpt_look_ahead) % len(x_list) # DONE
        self.next_index = lookahead_wpt_ind

        # 4. Calculate errors
        error_yaw = math.atan2(local_y_list[lookahead_wpt_ind], local_x_list[lookahead_wpt_ind])
        error_yaw = normalize_angle(error_yaw) # Normalize angle to [-pi, +pi]
        error_y   = local_y_list[lookahead_wpt_ind]
        return error_y, error_yaw

    def exit(self):
        rospy.loginfo('Halt received...')
        self.car_mode.data = False
        self.publish_command(0, 0) # neutral position
        self.pub_car_mde.publish(self.car_mode)

        rospy.loginfo('Quit received.')

def ros_node_main():
    # Define controller
    wpt_control = WaypointFollower()
    count = 0
    while not rospy.is_shutdown():
        if wpt_control.path_ready:
            #wpt_control.do_params()

            wpt_control.update_history_front(wpt_control.ego_x, wpt_control.ego_y, wpt_control.ego_yaw)

            # Lateral error calculation (cross-track error, yaw error)
            error_y, error_yaw = wpt_control.calc_error(sum(wpt_control.ego_x_history) / len(wpt_control.ego_x_history), \
                sum(wpt_control.ego_y_history) / len(wpt_control.ego_y_history), \
                sum(wpt_control.ego_yaw_history) / len(wpt_control.ego_yaw_history), \
                np.array(wpt_control.wpts_x), np.array(wpt_control.wpts_y), wpt_look_ahead=wpt_control.wpt_look_ahead)

            wpt_control.pub_target_pose.publish(wpt_control.wpts_poses[wpt_control.next_index])

            # Longitudinal error calculation (speed error)
            error_v = wpt_control.MAX_SPEED - wpt_control.ego_vx

            # Control

            steer_cmd = wpt_control.steer_control(error_yaw, error_y)
            speed_cmd = wpt_control.speed_control(error_v)
            wpt_control.update_history_end(error_yaw, error_y, error_v)

            # Publish command
            wpt_control.publish_command(steer_cmd, speed_cmd)
            #print(str(wpt_control.gains_steer) + " YAW: %0.2f Y: %0.2f STR: %0.2f" % (error_yaw, error_y, steer_cmd))

            # rospy.loginfo("C: (STR=%.2f, SPD=%.2f) E: (YAW=%.2f, Y=%.2f, V=%.2f)" \ # A: (YAW: %.2f, Y: %0.2f, V: %0.2f)
            #     %(steer_cmd, speed_cmd, error_yaw, error_y, error_v), #wpt_control.error_yaw_average, wpt_control.error_y_average, wpt_control.error_v_average))
    
        wpt_control.rate.sleep()

    wpt_control.exit()

if __name__ == '__main__':
    try:
        ros_node_main()
    except rospy.ROSInterruptException:
        pass
