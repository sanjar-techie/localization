#!/usr/bin/env python

from __future__ import print_function
import rospy
import sys
import os
import csv
import warnings
import traceback
import signal
from geometry_msgs.msg import PoseWithCovarianceStamped

received_msg = PoseWithCovarianceStamped()
do_spam = False
count_max = 100

class SigIntException(BaseException): pass

def stop(signal, frame):
    rospy.loginfo("You pressed ctrl-c")
    raise SigIntException

def callback(msg):
    global received_msg, do_spam
    if msg.header.frame_id == "map":
        rospy.loginfo("New pose received. Spamming...")
        received_msg = msg
        received_msg.header.frame_id = "spam"
        do_spam = True

def ros_node_main():
    global received_msg, do_spam, count_max
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, callback, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rospy.loginfo("Listening to /initalpose...")
    rate = rospy.Rate(20) # 10hz
    count = 0
    try:
        while not rospy.is_shutdown():
            if do_spam:
                pub.publish(received_msg)
                count = count + 1
                if count >= count_max:
                    rospy.loginfo("\tDone!")
                    do_spam = False
                    continue
            try:   
                rate.sleep()
            except:
                break

    except SigIntException:
        pass
    print("Quit received.")

if __name__ == '__main__':
    try:
        ros_node_main()
    except rospy.ROSInterruptException:
        pass
