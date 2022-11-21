#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32

def talker():
    pub = rospy.Publisher('MinjuHong', Int32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        hello_int = 20190697
        rospy.loginfo(hello_int)
        pub.publish(hello_int)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
