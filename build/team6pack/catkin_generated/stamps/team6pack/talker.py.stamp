#!/usr/bin/env python

from __future__ import print_function
import sys
import rospy
from std_msgs.msg import String

def talker(rate):
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(rate) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

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
        talker(rate)
    except rospy.ROSInterruptException:
        pass
